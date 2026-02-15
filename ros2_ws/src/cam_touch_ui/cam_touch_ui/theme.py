"""Qt theme helpers.

Lightweight, dependency-free dark theme for touch UIs on embedded Linux.
"""

from __future__ import annotations

from qtpy.QtGui import QColor, QPalette
from qtpy.QtWidgets import QApplication


def apply_dark_theme(app: QApplication) -> None:
    """Apply a consistent dark theme to the whole Qt application."""

    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(0x12, 0x12, 0x12))
    palette.setColor(QPalette.WindowText, QColor(0xE6, 0xE6, 0xE6))
    palette.setColor(QPalette.Base, QColor(0x10, 0x10, 0x10))
    palette.setColor(QPalette.AlternateBase, QColor(0x18, 0x18, 0x18))
    palette.setColor(QPalette.ToolTipBase, QColor(0x12, 0x12, 0x12))
    palette.setColor(QPalette.ToolTipText, QColor(0xE6, 0xE6, 0xE6))
    palette.setColor(QPalette.Text, QColor(0xE6, 0xE6, 0xE6))
    palette.setColor(QPalette.Button, QColor(0x2B, 0x2B, 0x2B))
    palette.setColor(QPalette.ButtonText, QColor(0xE6, 0xE6, 0xE6))
    palette.setColor(QPalette.BrightText, QColor(0xFF, 0x55, 0x55))
    palette.setColor(QPalette.Highlight, QColor(0x3D, 0x7A, 0xFF))
    palette.setColor(QPalette.HighlightedText, QColor(0x00, 0x00, 0x00))
    palette.setColor(QPalette.Link, QColor(0x65, 0xA3, 0xFF))
    app.setPalette(palette)

    app.setStyleSheet(
        """
        QWidget {
          background-color: #121212;
          color: #E6E6E6;
          font-size: 16px;
        }

        QPushButton {
          background: #2B2B2B;
          border: 1px solid #3A3A3A;
          border-radius: 12px;
          padding: 10px 16px;
        }
        QPushButton:hover {
          background: #333333;
        }
        QPushButton:pressed {
          background: #3A3A3A;
        }
        QPushButton:disabled {
          background: #1A1A1A;
          color: #707070;
          border: 1px solid #262626;
        }

        QToolTip {
          background-color: #1B1B1B;
          color: #E6E6E6;
          border: 1px solid #2B2B2B;
          padding: 6px;
        }
        """
    )
