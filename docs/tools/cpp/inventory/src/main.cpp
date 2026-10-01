// MA_Nav 文档流水线 · 证据提取器（inventory）
//
// 职责：给定仓库根、模块名与模块源码/头文件目录，提取该模块的**符号表**
//       （命名空间 / 类 / 结构体 / 枚举 / 函数 / 方法 / 成员变量）及其
//       定义文件与行号，输出为统一契约 JSON。
//
// 实现路径（经用户批准的 A 方案）：
//   通过 clangd 的 LSP 接口 textDocument/documentSymbol 取得符号。
//   clangd 使用 clang 的 C++ 前端，等价于 AST 级精度，但不产出目标文件、
//   不链接被文档化的仓库，也不需要 libclang 开发包。
//
// 不修改被文档化的仓库：本工具只读源码、只写 --out 指定的 JSON。
//
// 用法：
//   ma_nav_inventory --root <repo> --module <name>
//                    --sources <dir> [--sources <dir> ...]
//                    [--compdb-dir <dir>] [--clangd <path>]
//                    [--out <file.json>]

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

// ---------------------------------------------------------------- 小工具

std::string readFile(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return {};
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

// 从 .git/HEAD 解析当前 revision（不调用 git，避免任何仓库写操作风险）
std::string readGitRev(const std::string& root) {
    std::string head = readFile(root + "/.git/HEAD");
    while (!head.empty() && (head.back() == '\n' || head.back() == '\r')) head.pop_back();
    const std::string kRef = "ref: ";
    if (head.rfind(kRef, 0) == 0) {
        std::string ref = readFile(root + "/.git/" + head.substr(kRef.size()));
        while (!ref.empty() && (ref.back() == '\n' || ref.back() == '\r')) ref.pop_back();
        if (!ref.empty()) return ref.substr(0, 12);
    }
    return head.substr(0, 12);
}

std::string nowIso8601() {
    char buf[64];
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

bool hasSourceExtension(const fs::path& p) {
    static const std::set<std::string> exts = {
        ".cpp", ".cc", ".cxx", ".h", ".hpp", ".hxx", ".hh"};
    return exts.count(p.extension().string()) > 0;
}

// LSP SymbolKind -> 稳定字符串（契约用，不随 LSP 版本变化）
std::string symbolKindName(int kind) {
    switch (kind) {
        case 2:  return "module";
        case 3:  return "namespace";
        case 4:  return "package";
        case 5:  return "class";
        case 6:  return "method";
        case 7:  return "property";
        case 8:  return "field";
        case 9:  return "constructor";
        case 10: return "enum";
        case 11: return "interface";
        case 12: return "function";
        case 13: return "variable";
        case 14: return "constant";
        case 22: return "enum_member";
        case 23: return "struct";
        case 25: return "operator";
        case 26: return "type_parameter";
        default: return "other";
    }
}

// 这些 kind 参与限定名（qualified name）拼接
bool isScopeKind(int kind) {
    switch (kind) {
        case 3: case 5: case 10: case 11: case 23: return true;
        default: return false;
    }
}

// 需要写入符号表的 kind（命名空间本身不写入，只作为前缀）
bool isRecordedKind(int kind) {
    switch (kind) {
        case 5: case 6: case 8: case 9: case 10: case 12:
        case 13: case 14: case 22: case 23: case 25: return true;
        default: return false;
    }
}

// ---------------------------------------------------------------- LSP 客户端

class LspClient {
public:
    LspClient(const std::string& program, const std::vector<std::string>& args) {
        int in_pipe[2]{};   // child stdout -> parent
        int out_pipe[2]{};  // parent -> child stdin
        if (::pipe(in_pipe) != 0 || ::pipe(out_pipe) != 0) {
            throw std::runtime_error("pipe() 失败");
        }
        pid_ = ::fork();
        if (pid_ < 0) throw std::runtime_error("fork() 失败");
        if (pid_ == 0) {
            ::dup2(out_pipe[0], STDIN_FILENO);
            ::dup2(in_pipe[1], STDOUT_FILENO);
            ::close(in_pipe[0]); ::close(in_pipe[1]);
            ::close(out_pipe[0]); ::close(out_pipe[1]);
            std::vector<char*> argv;
            argv.push_back(const_cast<char*>(program.c_str()));
            for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
            argv.push_back(nullptr);
            ::execvp(program.c_str(), argv.data());
            std::perror("execvp clangd");
            ::_exit(127);
        }
        ::close(in_pipe[1]);
        ::close(out_pipe[0]);
        child_out_ = ::fdopen(in_pipe[0], "rb");
        child_in_ = ::fdopen(out_pipe[1], "wb");
        if (!child_out_ || !child_in_) throw std::runtime_error("fdopen() 失败");
    }

    ~LspClient() {
        if (child_in_) { std::fclose(child_in_); child_in_ = nullptr; }
        if (child_out_) { std::fclose(child_out_); child_out_ = nullptr; }
        if (pid_ > 0) {
            int status = 0;
            ::waitpid(pid_, &status, 0);
        }
    }

    void notify(const std::string& method, const json& params) {
        json msg = {{"jsonrpc", "2.0"}, {"method", method}, {"params", params}};
        send(msg);
    }

    json call(const std::string& method, const json& params) {
        int id = next_id_++;
        json msg = {{"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params}};
        send(msg);
        return awaitResponse(id);
    }

private:
    void send(const json& msg) {
        std::string body = msg.dump();
        std::string header = "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
        std::fwrite(header.data(), 1, header.size(), child_in_);
        std::fwrite(body.data(), 1, body.size(), child_in_);
        std::fflush(child_in_);
    }

    // 读取一条 LSP 消息；EOF 返回 nullopt
    bool readMessage(json& out) {
        std::string header;
        char c = 0;
        while (true) {
            size_t n = std::fread(&c, 1, 1, child_out_);
            if (n != 1) return false;
            header.push_back(c);
            if (header.size() >= 4 && header.compare(header.size() - 4, 4, "\r\n\r\n") == 0) break;
            if (header.size() > 8192) return false;
        }
        size_t pos = header.find("Content-Length:");
        if (pos == std::string::npos) return false;
        size_t eol = header.find("\r\n", pos);
        long len = std::strtol(header.substr(pos + 15, eol - pos - 15).c_str(), nullptr, 10);
        if (len <= 0) return false;
        std::string body(len, '\0');
        size_t got = std::fread(&body[0], 1, len, child_out_);
        if (got != static_cast<size_t>(len)) return false;
        out = json::parse(body, nullptr, false);
        return !out.is_discarded();
    }

    // 等待 id 对应的响应；途中遇到服务端请求必须应答，否则 clangd 会阻塞
    json awaitResponse(int id) {
        json msg;
        while (readMessage(msg)) {
            if (msg.contains("id") && msg.contains("method")) {
                handleServerRequest(msg);
                continue;
            }
            if (msg.value("id", -1) == id) {
                if (msg.contains("error")) {
                    throw std::runtime_error("LSP 错误: " + msg["error"].dump());
                }
                return msg.value("result", json());
            }
        }
        throw std::runtime_error("LSP 连接在等待响应时结束");
    }

    void handleServerRequest(const json& msg) {
        json result = nullptr;
        if (msg.value("method", "") == "workspace/configuration") {
            size_t n = msg.value("params", json::object()).value("items", json::array()).size();
            result = json::array();
            for (size_t i = 0; i < n; ++i) result.push_back(nullptr);
        }
        json reply = {{"jsonrpc", "2.0"}, {"id", msg["id"]}, {"result", result}};
        send(reply);
    }

    pid_t pid_ = -1;
    int next_id_ = 1;
    FILE* child_in_ = nullptr;
    FILE* child_out_ = nullptr;
};

// ---------------------------------------------------------------- 符号展开

struct SymbolRecord {
    std::string qname;
    std::string name;
    std::string kind;
    std::string file;
    int line = 0;
};

void flattenSymbols(const json& nodes, const std::string& prefix, const std::string& file,
                    const std::string& method, std::vector<SymbolRecord>& out) {
    for (const auto& node : nodes) {
        std::string name = node.value("name", "");
        if (name.empty()) continue;
        if (name.rfind("using ", 0) == 0) continue;  // "using namespace std" 之类不是符号
        int kind = node.value("kind", 0);
        const std::string kind_name =
            (method == "textDocument/documentSymbol") ? symbolKindName(kind) : "other";

        std::string qname = prefix.empty() ? name : prefix + "::" + name;
        const json& range = node.contains("range") ? node["range"] : node["location"]["range"];
        int line = range.value("start", json::object()).value("line", 0) + 1;

        if (isRecordedKind(kind)) {
            out.push_back({qname, name, kind_name, file, line});
        }

        if (node.contains("children") && node["children"].is_array()) {
            const std::string child_prefix = isScopeKind(kind) ? qname : prefix;
            flattenSymbols(node["children"], child_prefix, file, method, out);
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string root, module, compdb_dir, clangd = "clangd", out_path;
    std::vector<std::string> source_dirs;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* flag) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "缺少 " << flag << " 的参数\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--root") root = next("--root");
        else if (a == "--module") module = next("--module");
        else if (a == "--sources") source_dirs.push_back(next("--sources"));
        else if (a == "--compdb-dir") compdb_dir = next("--compdb-dir");
        else if (a == "--clangd") clangd = next("--clangd");
        else if (a == "--out") out_path = next("--out");
        else if (a == "-h" || a == "--help") {
            std::cout << "用法: ma_nav_inventory --root <repo> --module <name> "
                         "--sources <dir> [--compdb-dir <dir>] [--out <file>]\n";
            return 0;
        } else {
            std::cerr << "未知参数: " << a << "\n";
            return 2;
        }
    }
    if (root.empty() || module.empty() || source_dirs.empty()) {
        std::cerr << "必须提供 --root --module --sources\n";
        return 2;
    }
    if (compdb_dir.empty()) compdb_dir = root + "/build";

    // 收集模块文件（相对 root 输出）
    std::vector<std::string> files;
    for (const auto& rel : source_dirs) {
        fs::path dir = fs::path(root) / rel;
        if (!fs::exists(dir)) {
            std::cerr << "警告: 目录不存在 " << dir << "\n";
            continue;
        }
        for (const auto& entry : fs::recursive_directory_iterator(dir)) {
            if (!entry.is_regular_file()) continue;
            if (!hasSourceExtension(entry.path())) continue;
            files.push_back(fs::relative(entry.path(), root).string());
        }
    }
    std::sort(files.begin(), files.end());
    files.erase(std::unique(files.begin(), files.end()), files.end());
    if (files.empty()) {
        std::cerr << "未收集到任何源文件\n";
        return 1;
    }

    std::vector<SymbolRecord> symbols;
    std::vector<std::string> failed;

    try {
        std::vector<std::string> clangd_args = {
            "--background-index=0",
            "--pch-storage=memory",
            "--log=error",
            "--compile-commands-dir=" + compdb_dir,
        };
        LspClient client(clangd, clangd_args);

        json init_params = {
            {"processId", static_cast<int>(::getpid())},
            {"rootUri", "file://" + fs::absolute(root).string()},
            {"capabilities", {{"textDocument", {{"documentSymbol",
                {{"hierarchicalDocumentSymbolSupport", true}}}}}}},
        };
        json init_result = client.call("initialize", init_params);
        client.notify("initialized", json::object());

        const std::string generator =
            "clangd " + init_result.value("serverInfo", json::object()).value("version", "unknown") +
            " / documentSymbol";

        for (const auto& rel : files) {
            const std::string abs = (fs::path(root) / rel).string();
            const std::string text = readFile(abs);
            if (text.empty()) { failed.push_back(rel); continue; }
            const std::string uri = "file://" + abs;

            client.notify("textDocument/didOpen",
                          {{"textDocument", {{"uri", uri}, {"languageId", "cpp"},
                                             {"version", 1}, {"text", text}}}});
            json res;
            try {
                res = client.call("textDocument/documentSymbol",
                                  {{"textDocument", {{"uri", uri}}}});
            } catch (const std::exception& e) {
                failed.push_back(rel);
                client.notify("textDocument/didClose", {{"textDocument", {{"uri", uri}}}});
                continue;
            }
            if (res.is_array()) flattenSymbols(res, "", rel, "textDocument/documentSymbol", symbols);
            client.notify("textDocument/didClose", {{"textDocument", {{"uri", uri}}}});
        }

        // 去重
        std::sort(symbols.begin(), symbols.end(), [](const SymbolRecord& a, const SymbolRecord& b) {
            return std::tie(a.file, a.line, a.qname) < std::tie(b.file, b.line, b.qname);
        });
        symbols.erase(std::unique(symbols.begin(), symbols.end(),
                                  [](const SymbolRecord& a, const SymbolRecord& b) {
                                      return a.qname == b.qname && a.file == b.file &&
                                             a.line == b.line && a.kind == b.kind;
                                  }),
                      symbols.end());

        json out;
        out["schema"] = "ma_nav.evidence.v1";
        out["kind"] = "inventory";
        out["module"] = module;
        out["generator"] = generator;
        out["git_rev"] = readGitRev(root);
        out["generated_at"] = nowIso8601();
        out["root"] = fs::absolute(root).string();
        out["files"] = files;
        out["failed_files"] = failed;
        json arr = json::array();
        for (const auto& s : symbols) {
            arr.push_back({{"qname", s.qname}, {"name", s.name}, {"kind", s.kind},
                           {"file", s.file}, {"line", s.line}});
        }
        out["symbols"] = arr;

        const std::string text = out.dump(2);
        if (out_path.empty()) {
            std::cout << text << "\n";
        } else {
            fs::path p(out_path);
            if (p.has_parent_path()) fs::create_directories(p.parent_path());
            std::ofstream ofs(out_path);
            if (!ofs) { std::cerr << "无法写入 " << out_path << "\n"; return 1; }
            ofs << text << "\n";
        }
        std::cerr << "[inventory] module=" << module << " files=" << files.size()
                  << " symbols=" << symbols.size() << " failed=" << failed.size() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[inventory] 失败: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
