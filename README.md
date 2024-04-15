# Double-headed-Sentinel
该双头哨兵视觉代码基于rm_vision开发，在此十分感谢https://github.com/chenjunnn</br>
本项目的基本思路为通过发布两个detector节点，对装甲板进行识别后进行发送装甲板信息，中间会经过choose节点的选择转发到tracker，最后tracker再对装甲板信息进一步处理并向下发送到电控端</br>
</br>

    ！！！
    本项目默认所有的first或1表示云台左边的相机，所有的second或2表示云台右边的相机。请使用本开源的队伍注意该问题。由于相机驱动采用枚举的方式读取设备，因此确认相机左右与线的先后插拔顺序并无关系，而是和相机本身的硬件信息有关，需要采用本开源的队伍自行去进行测试以确定哪个是左哪个是右
    

相机标定信息应当存放在 hik_camer0a_first/config/camera_info.yaml 及 hik_camera_second/config/camera_info.yaml

相机曝光时间修改位于/rm_vision/rm_vision_bringup/config/node_params.yaml

哨兵坐标信息修改位于 /rm_gimbal_description/urdf/rm_gimbal.urdf.xacro

由于经常拿单独模块进行测试因此没有做太多的解耦，请谅解，有需要的可以直接在/rm_vision/rm_vision_bringup/config/node_params.yaml 处更改文件使其不必在以上文件修改

</br>

### 启动指令

    sudo chmod 777 /dev/ttyACM0
    source install/setup.bash
    ros2 launch rm_vision_bringup hardware.launch.py


### RM_VISION 入口（适用于步兵或英雄）

在此感谢深北莫北极熊战队同学的整理汇总

https://flowus.cn/lihanchen/share/b63e0d4e-10b5-4d3c-8af1-6387158f54c3
