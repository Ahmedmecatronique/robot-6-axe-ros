"""Couleurs et feuille de style (thème sombre) de l'IHM."""

OK = '#3fb950'
WARN = '#d29922'
ERR = '#f85149'
INFO = '#58a6ff'
MUTED = '#8b949e'
BG = '#0d1117'
PANEL = '#161b22'
BORDER = '#30363d'
TEXT = '#e6edf3'

# Couleurs des courbes / robots (r, g, b)
REAL_RGB = (0.20, 0.55, 0.95)      # robot mesuré : bleu, opaque
GHOST_RGB = (1.00, 0.60, 0.15)     # consigne : orange, translucide

STYLESHEET = f"""
QWidget {{ background-color: {BG}; color: {TEXT}; font-size: 13px; }}
QGroupBox {{ border: 1px solid {BORDER}; border-radius: 6px; margin-top: 10px; padding-top: 8px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {MUTED}; }}
QPushButton {{ background-color: {PANEL}; border: 1px solid {BORDER}; border-radius: 5px; padding: 6px 10px; }}
QPushButton:hover {{ border-color: {INFO}; }}
QPushButton:pressed {{ background-color: {BORDER}; }}
QPushButton:disabled {{ color: {MUTED}; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; }}
QTabBar::tab {{ background: {PANEL}; padding: 6px 14px; border: 1px solid {BORDER}; }}
QTabBar::tab:selected {{ background: {BG}; border-bottom-color: {BG}; }}
QTableWidget {{ gridline-color: {BORDER}; }}
QHeaderView::section {{ background-color: {PANEL}; border: 1px solid {BORDER}; padding: 3px; }}
QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox, QListWidget, QPlainTextEdit {{
    background-color: {PANEL}; border: 1px solid {BORDER}; border-radius: 4px; padding: 2px 4px; }}
QSlider::groove:horizontal {{ height: 6px; background: {BORDER}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: {INFO}; width: 14px; margin: -5px 0; border-radius: 7px; }}
QProgressBar {{ border: 1px solid {BORDER}; border-radius: 4px; text-align: center; }}
QProgressBar::chunk {{ background-color: {INFO}; }}
"""

ESTOP_STYLE = ("QPushButton { background-color: #c62828; color: white; font-size: 20px; font-weight: bold;"
               " border: 3px solid #ff8a80; border-radius: 10px; padding: 14px; }"
               "QPushButton:hover { background-color: #e53935; }"
               "QPushButton:pressed { background-color: #8e0000; }")


def badge(text_color: str, bg_color: str) -> str:
    """Style d'un badge (mode, état...)."""
    return (f'background-color: {bg_color}; color: {text_color}; font-weight: bold; '
            f'border-radius: 5px; padding: 3px 10px;')
