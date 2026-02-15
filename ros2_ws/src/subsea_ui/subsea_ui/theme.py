"""Qt theme helpers.

Lightweight, dependency-free dark theme for touch UIs on embedded Linux.

We intentionally avoid adding third-party theme packages (e.g. qdarkstyle) to
keep deployments simple on Raspberry Pi.
"""

from __future__ import annotations

from qtpy.QtGui import QColor, QPalette
from qtpy.QtWidgets import QApplication


def apply_dark_theme(app: QApplication) -> None:
    """Apply a consistent dark theme to the whole Qt application."""

    # Fusion style gives predictable widget rendering across platforms.
    app.setStyle("Fusion")

    # --- Palette (colors picked to be readable on small touchscreens)
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

    # --- QSS: fills in the gaps for tabs/scroll areas/etc.
    app.setStyleSheet(
        """
        QWidget {
          background-color: #121212;
          color: #E6E6E6;
          font-size: 16px;
        }

        QTabWidget::pane {
          border: 1px solid #2B2B2B;
          top: -1px;
        }

        QTabBar::tab {
          background: #1B1B1B;
          border: 1px solid #2B2B2B;
          border-bottom: none;
          padding: 10px 14px;
          margin-right: 4px;
          border-top-left-radius: 10px;
          border-top-right-radius: 10px;
          min-width: 120px;
        }
        QTabBar::tab:selected {
          background: #2A2A2A;
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

        QLineEdit, QSpinBox {
          background: #1B1B1B;
          border: 1px solid #3A3A3A;
          border-radius: 10px;
          padding: 8px;
          selection-background-color: #3D7AFF;
        }

        QPlainTextEdit {
          background: #0F0F0F;
          border: 1px solid #2B2B2B;
          border-radius: 10px;
          padding: 8px;
        }

        QScrollArea {
          border: none;
        }
        QScrollArea QWidget {
          background-color: #121212;
        }

        QToolTip {
          background-color: #1B1B1B;
          color: #E6E6E6;
          border: 1px solid #2B2B2B;
          padding: 6px;
        }
        """
    )
