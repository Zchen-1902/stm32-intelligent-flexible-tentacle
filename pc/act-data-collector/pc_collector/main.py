from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from app import APP_STYLE, MainWindow


def main() -> int:
    application = QApplication(sys.argv)
    application.setApplicationName("Flexible Tentacle ACT")
    application.setStyleSheet(APP_STYLE)
    window = MainWindow()
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())

