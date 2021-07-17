## parking_GUI

评价函数

acc_Assessment
停车姿态精度 

com_Assessment 
舒适性

rot_Assessment 
原地转向时长

T_Assessment 
泊车入位总时长

RiskAssess 
碰撞风险

eva 
最终得分计算


myvector
用于存储读取到信息的容器


回调函数

imuCallback 
读取加速度

parkingslotCallback  
读取车位信息（前后车位角点）

SteeringAngleCallback  
读取方向盘转角

Vehicle_pose2DCallback
读取车辆位姿信息（横、纵向位置，偏航角）  

velometerCallback  
读取车辆速度信息（横、纵向，计算车速）


主程序
parking_GUI


各文件需要在同一文件路径下
##
