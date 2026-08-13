"""Console entry point for the soldering dashboard."""

from __future__ import annotations

import os
import signal
import sys

from PyQt5 import QtWidgets
from PyQt5 import QtCore

from .main_window import DashboardWindow


def main(args: list[str] | None = None) -> int:
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    argv = sys.argv if args is None else [sys.argv[0], *args]
    app = QtWidgets.QApplication(argv)
    app.setApplicationName("Soldering Robot Dashboard")
    app.setOrganizationName("phorce")
    window = DashboardWindow()
    window.show()
    app.aboutToQuit.connect(window.shutdown_workers)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal_timer = QtCore.QTimer(app)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
