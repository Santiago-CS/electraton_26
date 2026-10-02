import sys
import os
import time
from PyQt6.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QLabel, QProgressBar
)
from PyQt6.QtGui import QPixmap, QFont
from PyQt6.QtCore import Qt, QTimer
from tools.telemetry_dashboard.ui.main_window import MainWindow

class TeamSplashScreen(QDialog):
    def __init__(self, logo_path: str):
        super().__init__(None, Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
        self.setStyleSheet("background-color: #000000; border: 2px solid #00e5ff; border-radius: 8px;")
        self.setFixedSize(650, 360)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(25, 25, 25, 20)
        layout.setSpacing(15)
        
        # 1. Imagen del Logo o Fallback tipográfico
        self.lbl_logo = QLabel()
        self.lbl_logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        loaded = False
        if os.path.exists(logo_path):
            pix = QPixmap(logo_path)
            if not pix.isNull():
                self.lbl_logo.setPixmap(
                    pix.scaled(580, 240, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                )
                loaded = True
        
        if not loaded:
            self.lbl_logo.setText("⚡ BE KEY ITESO RACING TEAM")
            self.lbl_logo.setStyleSheet("color: #00e5ff; font-size: 26px; font-weight: bold; letter-spacing: 2px;")
            
        layout.addWidget(self.lbl_logo, alignment=Qt.AlignmentFlag.AlignCenter)
        
        # 2. Etiqueta de estado de inicialización
        self.lbl_status = QLabel("Inicializando sistemas de telemetría y adquisición...")
        self.lbl_status.setStyleSheet("color: #90a4ae; font-size: 11px;")
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_status)
        
        # 3. Barra de progreso
        self.pbar = QProgressBar()
        self.pbar.setFixedHeight(12)
        self.pbar.setTextVisible(False)
        self.pbar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #1a237e;
                border-radius: 6px;
                background-color: #111317;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #76ff03, stop:0.5 #00e5ff, stop:1 #2979ff);
                border-radius: 5px;
            }
        """)
        layout.addWidget(self.pbar)

def run_app():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Intentar buscar el logo con diferentes nombres habituales
    base_dir = os.path.dirname(os.path.abspath(__file__))
    possible_names = [
        "logo.jpeg", "logo.jpg", "logo.png",
    ]
    logo_path = ""
    for name in possible_names:
        p = os.path.join(base_dir, "assets", name)
        if os.path.exists(p):
            logo_path = p
            break
            
    # Mostrar Splash
    splash = TeamSplashScreen(logo_path)
    splash.show()
    app.processEvents()

    # Progreso de arranque simulado mientras carga la UI
    steps = [
        (25, "Cargando controladores de bus CAN & transceptores..."),
        (50, "Cargando motor de dinámicas de chasis y GPS WGS84..."),
        (75, "Compilando widgets de trazado y curvas sincronizadas..."),
        (100, "¡Listo! Iniciando BE KEY Telemetry Suite...")
    ]
    for val, msg in steps:
        splash.pbar.setValue(val)
        splash.lbl_status.setText(msg)
        app.processEvents()
        time.sleep(0.25)

    window = MainWindow()
    splash.close()
    window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    run_app()