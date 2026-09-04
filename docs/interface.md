## planner

## 输入

###  goal: [x,y,yaw]
由决策发布

## planner

fsm ---> jps->post_process->opt->mpc->[vx,vy] //控制[x,y] 
    |             |
    |             --->折叠处理 得到云台折叠的位置，时间---->[发布goal_yaw,开始折叠的指令]
    |                                                           
    ---> yaw_controller->[Mode,goal_yaw发布给下位机的时间] //mode{ 小陀螺 yaw规划 默认 }
                |
                mode 上🔓  优先级： 折叠处理>决策（优先级高的可以抢占mode处理）
                
注：这里只处理yaw mode,以及什么时候发布
## 输出
### cmd: [vx,vy,yaw]
