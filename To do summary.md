
# 7.23 To do 
1. 真机实验stage1，
    1. 先测试各层通讯，用vision_double.launch只测试双目发送topic /vision_double/target_features
    2. 设置为zero模式，运行full.launch，运行simulink，记得打开config的false开关，测试simulink接受vision feature后能否输出关节topic /simulink/target_joints_velocities