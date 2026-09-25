from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from app import APP_STYLE, MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_STYLE)
    window = MainWindow(Path(__file__).resolve().parent)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

