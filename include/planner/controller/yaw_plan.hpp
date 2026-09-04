#pragma once

namespace control {
    /** 
    * 1.plan时过洞指定yaw角度
    * 2.决策---->yaw规划朝向,是否小陀螺 // 需要考虑当前位置-->目标位置的时间，再判断什么时候yaw规划
    * 3.默认
    */
    struct YawControlIntput{
        
        double goal_yaw;
    };
    class YawController {
        public:
            enum Mode{
                SPIN,//小陀螺
                PLAN,//yaw规划
                IDLE,//默认
            };
            auto control()->void;
        private:
            Mode mode_ = Mode::IDLE;
    };
}