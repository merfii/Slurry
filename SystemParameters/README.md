## 本文件夹内是程序正常运行所需的配置文件

配置文件在程序启动时载入。若任一缺失，则程序无法启动

- Settings.xml 全局参数 用于设定IP地址、串口名称、相机标示符、扫描帧率
- CameraParameterAB.xml 相机的内参数、外参数、畸变参数、激光平面参数 由标定程序自动保存
- PathAB.txt 标定相机内外参数时的机械臂轨迹
- PathLaser.txt 标定激光平面时的机械臂轨迹
- StereoParameter.yml 双目参数（暂未使用）
- ToolParameter.txt 导引筒（工具）参数