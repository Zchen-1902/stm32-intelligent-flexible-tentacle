"""ACT上位机的异步串口枚举、连接和重连服务。"""

from __future__ import annotations

from dataclasses import dataclass, replace

from PySide6.QtCore import (
    QIODeviceBase,
    QMetaObject,
    QObject,
    QThread,
    QTimer,
    Qt,
    Signal,
    Slot,
)
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo


@dataclass(frozen=True, slots=True)
class PortDescriptor:
    name: str
    description: str = ""
    manufacturer: str = ""
    serial_number: str = ""
    vendor_id: int | None = None
    product_id: int | None = None

    @property
    def display_name(self) -> str:
        label = self.description.strip() or self.manufacturer.strip() or "串口设备"
        return f"{self.name} · {label}"

    @property
    def identity(self) -> tuple[int | None, int | None, str]:
        return self.vendor_id, self.product_id, self.serial_number


@dataclass(slots=True)
class SerialSettings:
    port_name: str
    baud_rate: int = 115200
    data_bits: int = 8
    parity: str = "None"
    stop_bits: str = "1"
    flow_control: str = "None"


def enumerate_ports() -> list[PortDescriptor]:
    """返回Windows当前实际枚举的串口及其USB身份信息。"""

    result: list[PortDescriptor] = []
    for info in QSerialPortInfo.availablePorts():
        result.append(
            PortDescriptor(
                name=info.portName(),
                description=info.description(),
                manufacturer=info.manufacturer(),
                serial_number=info.serialNumber(),
                vendor_id=info.vendorIdentifier() if info.hasVendorIdentifier() else None,
                product_id=info.productIdentifier() if info.hasProductIdentifier() else None,
            )
        )
    return sorted(result, key=lambda item: _port_sort_key(item.name))


def _port_sort_key(name: str) -> tuple[str, int]:
    prefix = name.rstrip("0123456789")
    suffix = name[len(prefix) :]
    return prefix.upper(), int(suffix) if suffix.isdigit() else -1


class _SerialWorker(QObject):
    ports_scanned = Signal(object)
    received = Signal(bytes)
    transmitted = Signal(bytes)
    connection_changed = Signal(bool, str)
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

    def __init__(self) -> None:
        super().__init__()
        self._port = self._create_port()
        self._settings: SerialSettings | None = None
        self._identity: tuple[int | None, int | None, str] | None = None
        self._desired_open = False
        self._opening = False
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setSingleShot(True)
        self._reconnect_timer.timeout.connect(self._attempt_reconnect)

    def _create_port(self) -> QSerialPort:
        port = QSerialPort(self)
        port.readyRead.connect(self._read_available)
        port.errorOccurred.connect(self._on_error)
        return port

    def _replace_port(self) -> None:
        old = self._port
        old.blockSignals(True)
        if old.isOpen():
            old.close()
        old.deleteLater()
        self._port = self._create_port()

    @Slot()
    def scan_ports(self) -> None:
        self.ports_scanned.emit(enumerate_ports())

    @Slot(object)
    def open_port(self, settings: SerialSettings) -> None:
        self._reconnect_timer.stop()
        self._settings = replace(settings)
        self._identity = self._identity_for_name(settings.port_name)
        self._desired_open = True
        self._replace_port()
        if self._open_saved():
            self.connection_changed.emit(True, f"{self._port.portName()} 串口已打开")
            return
        message = self._port.errorString() or f"无法打开{settings.port_name}"
        self.connection_changed.emit(False, message)
        self.reconnecting.emit("等待串口设备重新出现")
        self._reconnect_timer.start(2000)

    @Slot()
    def close_port(self) -> None:
        name = self._port.portName() or (self._settings.port_name if self._settings else "串口")
        self._desired_open = False
        self._reconnect_timer.stop()
        self._replace_port()
        self.connection_changed.emit(False, f"{name} 已断开")

    @Slot(bytes)
    def write(self, payload: bytes) -> None:
        if not self._port.isOpen():
            self.error.emit("串口尚未打开")
            return
        accepted = self._port.write(payload)
        if accepted != len(payload):
            self.error.emit(self._port.errorString() or "串口未接受完整命令")
            return
        self.transmitted.emit(payload)

    def _identity_for_name(self, port_name: str) -> tuple[int | None, int | None, str] | None:
        for descriptor in enumerate_ports():
            if descriptor.name == port_name:
                return descriptor.identity
        return None

    def _resolved_port_name(self) -> str | None:
        if self._settings is None:
            return None
        ports = enumerate_ports()
        if any(port.name == self._settings.port_name for port in ports):
            return self._settings.port_name
        if self._identity is None:
            return None
        matches = [port.name for port in ports if port.identity == self._identity]
        return matches[0] if len(matches) == 1 else None

    def _open_saved(self) -> bool:
        if self._settings is None:
            return False
        port_name = self._resolved_port_name()
        if not port_name:
            return False
        settings = self._settings
        self._port.setPortName(port_name)
        self._port.setBaudRate(settings.baud_rate)
        self._port.setDataBits(self.DATA_BITS.get(settings.data_bits, QSerialPort.Data8))
        self._port.setParity(self.PARITY.get(settings.parity, QSerialPort.NoParity))
        self._port.setStopBits(self.STOP_BITS.get(settings.stop_bits, QSerialPort.OneStop))
        self._port.setFlowControl(self.FLOW.get(settings.flow_control, QSerialPort.NoFlowControl))
        self._opening = True
        try:
            opened = self._port.open(QIODeviceBase.ReadWrite)
        finally:
            self._opening = False
        if not opened:
            return False
        self._port.clear(QSerialPort.AllDirections)
        self._port.clearError()
        return True

    @Slot()
    def _attempt_reconnect(self) -> None:
        if not self._desired_open:
            return
        self._replace_port()
        if self._open_saved():
            self.connection_changed.emit(True, f"{self._port.portName()} 已重新连接")
            return
        self.reconnecting.emit("未找到原串口，继续等待")
        self._reconnect_timer.start(2000)

    @Slot()
    def _read_available(self) -> None:
        if self.sender() is not self._port:
            return
        payload = bytes(self._port.readAll())
        if payload:
            self.received.emit(payload)

    @Slot(QSerialPort.SerialPortError)
    def _on_error(self, error: QSerialPort.SerialPortError) -> None:
        if self.sender() is not self._port or self._opening:
            return
        if error in (QSerialPort.NoError, QSerialPort.NotOpenError):
            return
        message = self._port.errorString()
        fatal = {
            QSerialPort.DeviceNotFoundError,
            QSerialPort.PermissionError,
            QSerialPort.OpenError,
            QSerialPort.ResourceError,
            QSerialPort.UnsupportedOperationError,
        }
        if error in fatal:
            name = self._port.portName() or "串口"
            self._replace_port()
            self.connection_changed.emit(False, f"{name}连接已断开")
            if self._desired_open:
                self.reconnecting.emit(f"{name}异常，等待自动重连")
                self._reconnect_timer.start(1200)
        self.error.emit(message)


class SerialService(QObject):
    """主线程使用的串口服务外观；所有串口操作在工作线程中执行。"""

    ports_changed = Signal(object)
    received = Signal(bytes)
    transmitted = Signal(bytes)
    status_changed = Signal(bool, str)
    reconnecting = Signal(str)
    error = Signal(str)

    _scan_requested = Signal()
    _open_requested = Signal(object)
    _close_requested = Signal()
    _write_requested = Signal(bytes)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._thread = QThread(self)
        self._thread.setObjectName("ActSerialWorker")
        self._worker = _SerialWorker()
        self._worker.moveToThread(self._thread)

        self._scan_requested.connect(self._worker.scan_ports)
        self._open_requested.connect(self._worker.open_port)
        self._close_requested.connect(self._worker.close_port)
        self._write_requested.connect(self._worker.write)
        self._worker.ports_scanned.connect(self.ports_changed)
        self._worker.received.connect(self._on_received)
        self._worker.transmitted.connect(self._on_transmitted)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._worker.reconnecting.connect(self.reconnecting)
        self._worker.error.connect(self.error)

        self._is_open = False
        self._desired_open = False
        self.rx_bytes = 0
        self.tx_bytes = 0
        self._scan_timer = QTimer(self)
        self._scan_timer.setInterval(2000)
        self._scan_timer.timeout.connect(self._auto_scan)

        self._thread.start()
        self._scan_timer.start()
        self.refresh_ports()

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def desired_open(self) -> bool:
        return self._desired_open

    def refresh_ports(self) -> None:
        self._scan_requested.emit()

    def open(self, settings: SerialSettings) -> None:
        self._desired_open = True
        self._open_requested.emit(settings)

    def close(self) -> None:
        self._desired_open = False
        self._close_requested.emit()

    def write(self, payload: bytes) -> bool:
        if not self._is_open:
            self.error.emit("串口尚未打开")
            return False
        self._write_requested.emit(payload)
        return True

    def shutdown(self) -> None:
        self._desired_open = False
        self._scan_timer.stop()
        if self._thread.isRunning():
            QMetaObject.invokeMethod(self._worker, "close_port", Qt.BlockingQueuedConnection)
            self._thread.quit()
            self._thread.wait(1500)
        self._worker.deleteLater()

    def _auto_scan(self) -> None:
        if not self._is_open:
            self.refresh_ports()

    @Slot(bytes)
    def _on_received(self, payload: bytes) -> None:
        if not self._is_open:
            return
        self.rx_bytes += len(payload)
        self.received.emit(payload)

    @Slot(bytes)
    def _on_transmitted(self, payload: bytes) -> None:
        self.tx_bytes += len(payload)
        self.transmitted.emit(payload)

    @Slot(bool, str)
    def _on_connection_changed(self, opened: bool, message: str) -> None:
        self._is_open = opened
        self.status_changed.emit(opened, message)

