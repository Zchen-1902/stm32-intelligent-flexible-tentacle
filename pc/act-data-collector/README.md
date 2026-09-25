# Flexible Tentacle ACT

单触手ACT示教控制与Episode采集上位机。

当前已完成USART1单触手控制与ACT Episode采集链路：

- 固定三列主界面。
- 真实COM枚举与自动重连，默认 `2000000, 8N1`。
- 串口工作线程、拔出销毁和自动重连。
- ASCII `ACT PING/SELECT/STATUS/ENABLE/MOVE/HOME/STOP/CAPTURE`发送。
- `0xA55A`二进制ACK/STATUS/ERROR/FRAME流式解析。
- 只有PING ACK和STATUS均有效才显示H7已同步。
- 4198字节ACT_FRAME解析、ToF热力图和实际姿态降频显示。
- Episode内存缓存、序号缺口与质量统计、后台原子保存和退出恢复。
- 支持任意时长Episode，并在保存时自动检查训练字段、尺寸及有效窗口数量。
- 采集期间暂停常规STATUS轮询，避免占用H7帧发送时隙。

运行：

```powershell
cd F:\act_vla\flexible_tentacle_act\pc_collector
python main.py
```

测试：

```powershell
cd F:\act_vla\flexible_tentacle_act
python -m unittest discover -s tests -v
```

依赖：

```powershell
python -m pip install -r pc_collector\requirements.txt
```
