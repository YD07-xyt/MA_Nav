# 8.31
1. 加载ma opt的参数，修改mpc的参数
2. 判断什么时候低头（隧道）
3. 优化架构


1. astar jps [Done]

2. minco opt [Done]

3. mpc  [Done]  

4. fsm  [Done]

5. ros2  分组多线程 [Done]

6. map 语义 隧道   

7. opt+ mpc 考虑yaw,wz  xy ,yaw分开优化 [X] ---> x,y,yaw联合优化 [x] ---> 单独的yaw控制器

8. post_processing 分派时间时考虑隧道前低头-->发布低头指令

9. 地图处理 直接处理点云  聚类动态点云跟踪，过滤坡度，悬空障碍物  

10. 优化架构，细节
