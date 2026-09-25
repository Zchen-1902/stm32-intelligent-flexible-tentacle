from __future__ import annotations

from dataclasses import dataclass, replace

from PySide6.QtCore import QObject, QIODeviceBase, QTimer, Signal
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo


@dataclass(slots=True)
class SerialSettings:
    port_name: str
    baud_rate: int = 921600
    data_bits: int = 8
    parity: str = "None"
    stop_bits: str = "1"
    flow_control: str = "None"


class SerialService(QObject):
    received = Signal(bytes)
    transmitted = Signal(bytes)
    status_changed = Signal(bool, str)
    reconnecting = Signal(str)
    error = Signal(str)

    DATA_BITS = {
        5: QSerialPort.Data5,
        6: QSerialPort.Data6,
        7: QSerialPort.Data7,
        8: QSerialPort.Data8,
    }
    PARITY = {
        "None": QSerialPort.NoParity,
        "Even": QSerialPort.EvenParity,
        "Odd": QSerialPort.OddParity,
        "Mark": QSerialPort.MarkParity,
        "Space": QSerialPort.SpaceParity,
    }
    STOP_BITS = {
        "1": QSerialPort.OneStop,
        "1.5": QSerialPort.OneAndHalfStop,
        "2": QSerialPort.TwoStop,
    }
    FLOW = {
        "None": QSerialPort.NoFlowControl,
        "RTS/CTS": QSerialPort.HardwareControl,
        "XON/XOFF": QSerialPort.SoftwareControl,
    }

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.port = self._create_port()
        self._reported_open = False
        self._desired_open = False
        self._recovering = False
        self._opening = False
        self._saved_settings: SerialSettings | None = None
        self._port_identity: tuple[int | None, int | None, str] | None = None
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setSingleShot(True)
        self._reconnect_timer.timeout.connect(self._attempt_reconnect)
        self.rx_bytes = 0
        self.tx_bytes = 0

    @property
    def desired_open(self) -> bool:
        """用户是否仍希望保持串口连接；主动关闭后自动重连必须停止。"""
        return self._desired_open

    def _create_port(self) -> QSerialPort:
        """创建全新的Qt串口对象并连接信号。"""
        port = QSerialPort(self)
        port.readyRead.connect(self._read_available)
        port.errorOccurred.connect(self._on_error)
        return port

    def _replace_port(self) -> None:
        """关闭并销毁旧串口对象，避免复用异常驱动句柄。"""
        old_port = self.port
        old_port.blockSignals(True)
        if old_port.isOpen():
            old_port.close()
        old_port.deleteLater()
        self.port = self._create_port()

    @staticmethod
    def _identity_for_name(port_name: str) -> tuple[int | None, int | None, str] | None:
        """读取接收器身份，使重新枚举后即使COM号变化也能找到原设备。"""
        for info in QSerialPortInfo.availablePorts():
            if info.portName() != port_name:
                continue
            vendor = info.vendorIdentifier() if info.hasVendorIdentifier() else None
            product = info.productIdentifier() if info.hasProductIdentifier() else None
            return vendor, product, info.serialNumber() or ""
        return None

    def _resolved_port_name(self) -> str | None:
        """优先使用原COM，其次按USB身份匹配重新枚举后的端口。"""
        if self._saved_settings is None:
            return None
        infos = list(QSerialPortInfo.availablePorts())
        if any(info.portName() == self._saved_settings.port_name for info in infos):
            return self._saved_settings.port_name
        if self._port_identity is None:
            return None
        vendor, product, serial = self._port_identity
        candidates: list[str] = []
        for info in infos:
            info_vendor = info.vendorIdentifier() if info.hasVendorIdentifier() else None
            info_product = info.productIdentifier() if info.hasProductIdentifier() else None
            if info_vendor != vendor or info_product != product:
                continue
            if serial and (info.serialNumber() or "") != serial:
                continue
            candidates.append(info.portName())
        return candidates[0] if len(candidates) == 1 else None

    @staticmethod
    def available_ports() -> list[str]:
        return [port.portName() for port in QSerialPortInfo.availablePorts()]

    def open(self, settings: SerialSettings) -> bool:
        self._reconnect_timer.stop()
        self._desired_open = False
        self._recovering = False
        self._saved_settings = replace(settings)
        self._port_identity = self._identity_for_name(settings.port_name)
        self._replace_port()
        opened = self._open_saved_port()
        self._desired_open = opened
        self._reported_open = opened
        message = f"{settings.port_name} 已连接" if opened else self.port.errorString()
        self.status_changed.emit(opened, message)
        return opened

    def _open_saved_port(self) -> bool:
        """使用保存的配置打开当前新串口对象。"""
        if self._saved_settings is None:
            return False
        port_name = self._resolved_port_name()
        if not port_name:
            return False
        settings = self._saved_settings
        self.port.setPortName(port_name)
        self.port.setBaudRate(settings.baud_rate)
        self.port.setDataBits(self.DATA_BITS.get(settings.data_bits, QSerialPort.Data8))
        self.port.setParity(self.PARITY.get(settings.parity, QSerialPort.NoParity))
        self.port.setStopBits(self.STOP_BITS.get(settings.stop_bits, QSerialPort.OneStop))
        self.port.setFlowControl(self.FLOW.get(settings.flow_control, QSerialPort.NoFlowControl))
        self._opening = True
        try:
            opened = self.port.open(QIODeviceBase.ReadWrite)
        finally:
            self._opening = False
        if not opened:
            return False
        # 清掉旧会话残留，同时把无流控串口的控制线恢复到确定状态。
        self.port.blockSignals(True)
        try:
            self.port.clear(QSerialPort.AllDirections)
            if settings.flow_control == "None":
                self.port.setDataTerminalReady(False)
                self.port.setRequestToSend(False)
            self.port.clearError()
        finally:
            self.port.blockSignals(False)
        return True

    def force_reconnect(self, reason: str = "H7无响应") -> bool:
        """销毁异常串口对象并后台重建；只在用户保持开启意图时执行。"""
        if not self._desired_open or self._saved_settings is None:
            return False
        if self._recovering and self._reconnect_timer.isActive():
            return True
        self._reported_open = False
        self._recovering = True
        self._replace_port()
        self.reconnecting.emit(reason)
        self._reconnect_timer.start(800)
        return True

    def _attempt_reconnect(self) -> None:
        """尝试重建原接收器，失败时低频重试，直到用户主动关闭。"""
        if not self._desired_open or self._saved_settings is None:
            return
        if self._open_saved_port():
            self._reported_open = True
            self._recovering = False
            self.status_changed.emit(True, f"{self.port.portName()} 已重新连接")
            return
        self._replace_port()
        self._reconnect_timer.start(2000)

    def close(self) -> None:
        name = self.port.portName()
        self._desired_open = False
        self._recovering = False
        self._reconnect_timer.stop()
        self._reported_open = False
        self._replace_port()
        self.status_changed.emit(False, f"{name or '串口'} 已关闭")

    def write(self, payload: bytes) -> bool:
        if not self.port.isOpen():
            self.error.emit("串口尚未打开")
            return False
        count = self.port.write(payload)
        if count < 0:
            self.error.emit(self.port.errorString())
            return False
        self.tx_bytes += len(payload)
        self.transmitted.emit(payload)
        return True

    def _read_available(self) -> None:
        source = self.sender()
        if source is not self.port:
            return
        payload = bytes(self.port.readAll())
        if not payload:
            return
        self.rx_bytes += len(payload)
        self.received.emit(payload)

    def _on_error(self, error: QSerialPort.SerialPortError) -> None:
        source = self.sender()
        if source is not None and source is not self.port:
            return
        if error in (QSerialPort.NoError, QSerialPort.NotOpenError):
            return
        if self._opening:
            return

        message = self.port.errorString()
        fatal_errors = (
            QSerialPort.DeviceNotFoundError,
            QSerialPort.PermissionError,
            QSerialPort.OpenError,
            QSerialPort.ResourceError,
            QSerialPort.UnsupportedOperationError,
        )
        if error in fatal_errors:
            name = self.port.portName()
            was_connected = self._reported_open or self.port.isOpen()
            if self._desired_open and was_connected:
                self.force_reconnect(f"{name or '串口'}异常，正在自动重连")
            elif was_connected:
                self._reported_open = False
                self._replace_port()
                self.status_changed.emit(
                    False,
                    f"{name or '串口'} 连接已断开：{message}",
                )

        self.error.emit(message)
