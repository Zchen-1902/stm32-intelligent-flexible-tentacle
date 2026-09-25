# 四触手控制台原型

这是独立的Windows桌面原型，不修改H7/G4固件。

## 运行

使用现有设计工具虚拟环境：

```powershell
E:\sobot\Open-Spiral-Robots-main\design-tool\.venv\Scripts\python.exe PC_Tentacle_Controller\main.py
```

默认启动模拟模式。打开真实串口后才会发送串口数据。

## 当前原型范围

- 四触手2x2工作区和缩小窗口布局基础。
- 纯Qt轻量伪三维姿态显示，无需MuJoCo和OpenGL。
- VOFA JSON命令导入和分组按钮。
- 每条命令独立保存上一次输入内容。
- 串口配置、ASCII/HEX、CRLF和可展开接收窗口。
- 参数锁定、当前对象高亮、回正、模式和模拟响应。
