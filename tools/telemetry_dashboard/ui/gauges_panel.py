from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QLabel, QFrame
from PyQt6.QtCore import Qt

class GaugesPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        
        self.setStyleSheet("""
            QFrame#TelemetryCard {
                background-color: #1a1d24;
                border-radius: 8px;
                border: 1px solid #2a2e39;
            }
            QLabel {
                color: #e0e0e0;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QLabel#ValueLabel {
                font-size: 20px;
                font-weight: bold;
            }
            QLabel#TitleLabel {
                font-size: 11px;
                color: #8b949e;
                text-transform: uppercase;
            }
        """)
        
        card = QFrame()
        card.setObjectName("TelemetryCard")
        grid = QGridLayout(card)
        grid.setSpacing(12)
        
        self.lbl_speed = self._create_field(grid, "Velocidad", "0.0 km/h", "#00e5ff", 0, 0)
        self.lbl_rpm = self._create_field(grid, "Motor RPM", "0 RPM", "#ffffff", 0, 1)
        self.lbl_throttle = self._create_field(grid, "Acelerador", "0 %", "#00e676", 1, 0)
        self.lbl_brake = self._create_field(grid, "Freno", "0.0 bar", "#ff1744", 1, 1)
        self.lbl_ax = self._create_field(grid, "Ax (Long)", "0.00 g", "#ffea00", 2, 0)
        self.lbl_ay = self._create_field(grid, "Ay (Lat)", "0.00 g", "#e040fb", 2, 1)
        self.lbl_mot_temp = self._create_field(grid, "Temp Motor", "0.0 °C", "#ff9100", 3, 0)
        self.lbl_bat_temp = self._create_field(grid, "Temp Pack", "0.0 °C", "#00b0ff", 3, 1)
        self.lbl_voltage = self._create_field(grid, "Voltaje Pack", "0.0 V", "#b388ff", 4, 0)
        self.lbl_current = self._create_field(grid, "Corriente Pack", "0.0 A", "#ff5252", 4, 1)
        
        layout.addWidget(card)
        layout.addStretch()

    def _create_field(self, grid, title, default_val, color_hex, row, col):
        t_lbl = QLabel(title)
        t_lbl.setObjectName("TitleLabel")
        v_lbl = QLabel(default_val)
        v_lbl.setObjectName("ValueLabel")
        v_lbl.setStyleSheet(f"color: {color_hex};")
        
        box = QVBoxLayout()
        box.setSpacing(2)
        box.addWidget(t_lbl)
        box.addWidget(v_lbl)
        grid.addLayout(box, row, col)
        return v_lbl

    def update_values(self, row_data):
        self.lbl_speed.setText(f"{row_data['speed_kmh']:.1f} km/h")
        self.lbl_rpm.setText(f"{int(row_data['motor_rpm'])} RPM")
        self.lbl_throttle.setText(f"{row_data['throttle_pct']:.0f} %")
        self.lbl_brake.setText(f"{row_data['brake_bar']:.1f} bar")
        self.lbl_ax.setText(f"{row_data['ax_g']:.2f} g")
        self.lbl_ay.setText(f"{row_data['ay_g']:.2f} g")
        self.lbl_mot_temp.setText(f"{row_data['motor_temp_c']:.1f} °C")
        self.lbl_bat_temp.setText(f"{row_data['battery_temp_c']:.1f} °C")
        self.lbl_voltage.setText(f"{row_data['pack_voltage_v']:.1f} V")
        self.lbl_current.setText(f"{row_data['pack_current_a']:.1f} A")