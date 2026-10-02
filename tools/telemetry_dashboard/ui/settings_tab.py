from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QComboBox, QSpinBox, 
    QPushButton, QColorDialog, QLabel, QGroupBox
)
from PyQt6.QtCore import pyqtSignal

# Diccionario de Internacionalización
LANGUAGES = {
    "es": {
        "title": "Configuración del Sistema",
        "lang_select": "Idioma de Interfaz:",
        "theme_group": "Colores de Canales de Telemetría",
        "speed_col": "Color Velocidad (km/h):",
        "throttle_col": "Color Acelerador (%):",
        "brake_col": "Color Freno (bar):",
        "ax_col": "Color G-Force Longitudinal:",
        "ay_col": "Color G-Force Lateral:",
        "track_map_group": "Mapa de Pista GPS",
        "colormap_label": "Paleta de Mapa de Calor:",
        "line_thickness": "Grosor de Trazado de Curvas:",
        "btn_apply": "Aplicar Cambios"
    },
    "en": {
        "title": "System Settings",
        "lang_select": "Interface Language:",
        "theme_group": "Telemetry Channel Colors",
        "speed_col": "Speed Color (km/h):",
        "throttle_col": "Throttle Color (%):",
        "brake_col": "Brake Color (bar):",
        "ax_col": "Longitudinal G-Force Color:",
        "ay_col": "Lateral G-Force Color:",
        "track_map_group": "GPS Track Map",
        "colormap_label": "Heatmap Colormap:",
        "line_thickness": "Curve Line Thickness:",
        "btn_apply": "Apply Changes"
    }
}

class SettingsTab(QWidget):
    settings_changed = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.config = {
            "lang": "es",
            "speed_color": "#00e5ff",
            "throttle_color": "#00e676",
            "brake_color": "#ff1744",
            "ax_color": "#ffea00",
            "ay_color": "#e040fb",
            "line_width": 2,
            "colormap": "turbo"
        }
        self._init_ui()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # 1. Grupo Idioma
        grp_lang = QGroupBox("Idioma / Language")
        form_lang = QFormLayout(grp_lang)
        self.combo_lang = QComboBox()
        self.combo_lang.addItems(["Español", "English"])
        form_lang.addRow("Seleccionar / Select:", self.combo_lang)
        main_layout.addWidget(grp_lang)
        
        # 2. Grupo Colores
        grp_colors = QGroupBox("Colores de Telemetría")
        form_colors = QFormLayout(grp_colors)
        
        self.btn_col_speed = self._create_color_btn(self.config["speed_color"])
        self.btn_col_throttle = self._create_color_btn(self.config["throttle_color"])
        self.btn_col_brake = self._create_color_btn(self.config["brake_color"])
        
        form_colors.addRow("Velocidad:", self.btn_col_speed)
        form_colors.addRow("Acelerador:", self.btn_col_throttle)
        form_colors.addRow("Freno:", self.btn_col_brake)
        
        self.spin_width = QSpinBox()
        self.spin_width.setRange(1, 5)
        self.spin_width.setValue(self.config["line_width"])
        form_colors.addRow("Grosor de Línea:", self.spin_width)
        
        main_layout.addWidget(grp_colors)
        
        # 3. Grupo Mapa GPS
        grp_map = QGroupBox("Opciones de Mapa GPS")
        form_map = QFormLayout(grp_map)
        self.combo_colormap = QComboBox()
        self.combo_colormap.addItems(["turbo", "viridis", "inferno", "plasma"])
        form_map.addRow("Paleta de Velocidad:", self.combo_colormap)
        main_layout.addWidget(grp_map)
        
        # Botón Guardar
        self.btn_apply = QPushButton("Guardar y Aplicar")
        self.btn_apply.setStyleSheet("background-color: #00e5ff; color: #000; font-weight: bold; padding: 10px; border-radius: 4px;")
        self.btn_apply.clicked.connect(self._apply)
        main_layout.addWidget(self.btn_apply)
        main_layout.addStretch()

    def _create_color_btn(self, initial_hex):
        btn = QPushButton(initial_hex)
        btn.setStyleSheet(f"background-color: {initial_hex}; color: #000; font-weight: bold; border-radius: 4px; padding: 4px;")
        btn.clicked.connect(lambda: self._pick_color(btn))
        return btn

    def _pick_color(self, btn):
        color = QColorDialog.getColor()
        if color.isValid():
            hex_c = color.name()
            btn.setText(hex_c)
            btn.setStyleSheet(f"background-color: {hex_c}; color: #000; font-weight: bold; border-radius: 4px; padding: 4px;")

    def _apply(self):
        self.config["lang"] = "es" if self.combo_lang.currentIndex() == 0 else "en"
        self.config["speed_color"] = self.btn_col_speed.text()
        self.config["throttle_color"] = self.btn_col_throttle.text()
        self.config["brake_color"] = self.btn_col_brake.text()
        self.config["line_width"] = self.spin_width.value()
        self.config["colormap"] = self.combo_colormap.currentText()
        self.settings_changed.emit(self.config)