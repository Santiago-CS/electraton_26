import numpy as np
import pandas as pd
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QSlider, QFrame,
    QSplitter, QScrollArea, QMainWindow, QPushButton
)
from PyQt6.QtCore import Qt, pyqtSignal

from tools.telemetry_dashboard.ui.track_map import TrackMapWidget
from tools.telemetry_dashboard.ui.gg_diagram import GGDiagramWidget
from tools.telemetry_dashboard.ui.gauges_panel import GaugesPanel

CHANNELS = {
    "Velocidad (km/h)": ("speed_kmh", "km/h", "Velocidad"),
    "Consumo Corriente (A)": ("pack_current_a", "A", "Corriente"),
    "Potencia (kW)": ("power_kw", "kW", "Potencia"),
    "Aceleración Long (Ax)": ("ax_g", "g", "Aceleración Ax"),
    "Fuerza Lateral (Ay)": ("ay_g", "g", "Fuerza Lateral Ay"),
    "Acelerador (%)": ("throttle_pct", "%", "Acelerador"),
    "Freno (bar)": ("brake_bar", "bar", "Freno"),
    "Voltaje Pack (V)": ("pack_voltage_v", "V", "Voltaje"),
    "Temp Motor (°C)": ("motor_temp_c", "°C", "Temp Motor")
}

class DetachedWindow(QMainWindow):
    closed = pyqtSignal()

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(1300, 700)
        self.setStyleSheet("background-color: #0d0f12; color: #ffffff;")

    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)


class ComparisonTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.full_df = None
        self.df_a = None
        self.df_b = None
        self.lap_summary = {}
        self.max_common_dist = 550.0

        self.win_graphs = None
        self.win_cars = None

        self._init_ui()

    def _init_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(6)

        # 1. Barra de Controles Multicanal y Pop-ups
        header = QFrame()
        header.setStyleSheet("background-color: #1a1d24; border-radius: 6px; padding: 6px;")
        h_layout = QHBoxLayout(header)

        # Canal A
        lbl_a = QLabel("Vuelta A:")
        lbl_a.setStyleSheet("color: #00e5ff; font-weight: bold;")
        self.combo_lap_a = QComboBox()
        self.combo_lap_a.setStyleSheet("background-color: #111317; color: #00e5ff; font-weight: bold; border: 1px solid #00e5ff; padding: 3px 6px; border-radius: 4px;")
        self.combo_lap_a.currentIndexChanged.connect(self._on_inputs_changed)

        self.combo_chan_a = QComboBox()
        self.combo_chan_a.setStyleSheet("background-color: #111317; color: #ffffff; border: 1px solid #555; padding: 3px 6px; border-radius: 4px;")
        self.combo_chan_a.addItems(list(CHANNELS.keys()))
        self.combo_chan_a.setCurrentText("Velocidad (km/h)")
        self.combo_chan_a.currentIndexChanged.connect(self._on_inputs_changed)

        # Canal B
        lbl_b = QLabel("vs  Vuelta B:")
        lbl_b.setStyleSheet("color: #ff9100; font-weight: bold; margin-left: 12px;")
        self.combo_lap_b = QComboBox()
        self.combo_lap_b.setStyleSheet("background-color: #111317; color: #ff9100; font-weight: bold; border: 1px solid #ff9100; padding: 3px 6px; border-radius: 4px;")
        self.combo_lap_b.currentIndexChanged.connect(self._on_inputs_changed)

        self.combo_chan_b = QComboBox()
        self.combo_chan_b.setStyleSheet("background-color: #111317; color: #ffffff; border: 1px solid #555; padding: 3px 6px; border-radius: 4px;")
        self.combo_chan_b.addItems(list(CHANNELS.keys()))
        self.combo_chan_b.setCurrentText("Velocidad (km/h)")
        self.combo_chan_b.currentIndexChanged.connect(self._on_inputs_changed)

        # Modo Overlay vs Split
        lbl_mode = QLabel("Modo:")
        lbl_mode.setStyleSheet("color: #e0e0e0; font-weight: bold; margin-left: 12px;")
        self.combo_mode = QComboBox()
        self.combo_mode.setStyleSheet("background-color: #111317; color: #ffffff; border: 1px solid #555; padding: 3px 6px; border-radius: 4px;")
        self.combo_mode.addItems(["Superpuestas (Overlay)", "Separadas (Split)"])
        self.combo_mode.currentIndexChanged.connect(self._switch_mode_visibility)

        # Botones Pop-up
        self.btn_pop_graphs = QPushButton("🗗 Pop-up Gráficas")
        self.btn_pop_graphs.setStyleSheet("background-color: #004d40; color: #a7ffeb; font-weight: bold; padding: 4px 10px; border-radius: 4px; margin-left: 10px;")
        self.btn_pop_graphs.clicked.connect(self._toggle_popup_graphs)

        self.btn_pop_cars = QPushButton("🗗 Pop-up GPS/Autos")
        self.btn_pop_cars.setStyleSheet("background-color: #37474f; color: #00e5ff; font-weight: bold; padding: 4px 10px; border-radius: 4px; margin-left: 6px;")
        self.btn_pop_cars.clicked.connect(self._toggle_popup_cars)

        self.lbl_delta_summary = QLabel("")
        self.lbl_delta_summary.setStyleSheet("color: #ffffff; font-weight: bold; font-size: 12px; margin-left: 14px;")

        h_layout.addWidget(lbl_a)
        h_layout.addWidget(self.combo_lap_a)
        h_layout.addWidget(self.combo_chan_a)
        h_layout.addWidget(lbl_b)
        h_layout.addWidget(self.combo_lap_b)
        h_layout.addWidget(self.combo_chan_b)
        h_layout.addWidget(lbl_mode)
        h_layout.addWidget(self.combo_mode)
        h_layout.addWidget(self.btn_pop_graphs)
        h_layout.addWidget(self.btn_pop_cars)
        h_layout.addWidget(self.lbl_delta_summary)
        h_layout.addStretch()
        root_layout.addWidget(header)

        # 2. Splitter Vertical
        self.splitter = QSplitter(Qt.Orientation.Vertical)
        self.splitter.setStyleSheet("QSplitter::handle { background-color: #2a2e39; height: 5px; }")

        # --- SECCIÓN SUPERIOR: GRÁFICAS PRE-CREADAS PERMANENTES ---
        self.graph_view = pg.GraphicsLayoutWidget()
        self.graph_view.setBackground('#111317')
        self.graph_view.setMinimumHeight(280)

        # 2.1 Plot Overlay (Fila 0)
        self.plot_overlay = self.graph_view.addPlot(row=0, col=0)
        self.plot_overlay.setLabel('left', 'Velocidad', units='km/h')
        self.plot_overlay.showGrid(x=True, y=True, alpha=0.3)
        self.legend_overlay = self.plot_overlay.addLegend(offset=(20, 15))
        self.legend_overlay.setBrush(pg.mkBrush(20, 24, 30, 220))
        self.legend_overlay.setPen(pg.mkPen('#2a2e39', width=1))
        self.curve_overlay_a = self.plot_overlay.plot(pen=pg.mkPen('#00e5ff', width=2), name="A")
        self.curve_overlay_b = self.plot_overlay.plot(pen=pg.mkPen('#ff9100', width=2), name="B")

        # 2.2 Plots Split (Filas 1 y 2)
        self.plot_split_a = self.graph_view.addPlot(row=1, col=0)
        self.plot_split_a.setLabel('left', 'A: Canal', units='')
        self.plot_split_a.showGrid(x=True, y=True, alpha=0.3)
        self.curve_split_a = self.plot_split_a.plot(pen=pg.mkPen('#00e5ff', width=2))
        self.plot_split_a.setXLink(self.plot_overlay)

        self.plot_split_b = self.graph_view.addPlot(row=2, col=0)
        self.plot_split_b.setLabel('left', 'B: Canal', units='')
        self.plot_split_b.showGrid(x=True, y=True, alpha=0.3)
        self.curve_split_b = self.plot_split_b.plot(pen=pg.mkPen('#ff9100', width=2))
        self.plot_split_b.setXLink(self.plot_overlay)

        # 2.3 Plot Delta (Fila 3)
        self.plot_delta = self.graph_view.addPlot(row=3, col=0)
        self.plot_delta.setLabel('left', 'Δ Time', units='s')
        self.plot_delta.setLabel('bottom', 'Distancia Recorrida', units='m')
        self.plot_delta.showGrid(x=True, y=True, alpha=0.3)
        self.plot_delta.setXLink(self.plot_overlay)
        self.zero_line = self.plot_delta.plot([0, 600], [0, 0], pen=pg.mkPen('#555555', width=1))
        self.curve_delta = self.plot_delta.plot(pen=pg.mkPen('#ffea00', width=2))

        # Cursor Vertical en Gráfica Overlay
        self.v_line = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('#ffffff', width=2, style=pg.QtCore.Qt.PenStyle.DashLine))
        self.plot_overlay.addItem(self.v_line, ignoreBounds=True)
        self.v_line.sigPositionChanged.connect(self._on_cursor_line_moved)

        self.splitter.addWidget(self.graph_view)

        # --- SECCIÓN INFERIOR: AUTOS LADO A LADO ---
        self.bottom_scroll = QScrollArea()
        self.bottom_scroll.setWidgetResizable(True)
        self.bottom_scroll.setStyleSheet("QScrollArea { border: none; background: #0d0f12; }")

        self.car_container = QWidget()
        car_layout = QHBoxLayout(self.car_container)
        car_layout.setContentsMargins(6, 6, 6, 6)
        car_layout.setSpacing(14)

        # Lado Vuelta A
        box_a = QVBoxLayout()
        lbl_panel_a = QLabel("🏁 VUELTA A")
        lbl_panel_a.setStyleSheet("color: #00e5ff; font-weight: bold; font-size: 13px;")
        lbl_panel_a.setAlignment(Qt.AlignmentFlag.AlignCenter)
        box_a.addWidget(lbl_panel_a)

        row_a = QHBoxLayout()
        self.map_a = TrackMapWidget(title="GPS Vuelta A")
        self.map_a.setMinimumHeight(240)
        self.gg_a = GGDiagramWidget()
        self.gg_a.setMinimumHeight(240)
        row_a.addWidget(self.map_a, stretch=1)
        row_a.addWidget(self.gg_a, stretch=1)
        box_a.addLayout(row_a, stretch=2)

        self.gauges_a = GaugesPanel()
        box_a.addWidget(self.gauges_a, stretch=1)
        car_layout.addLayout(box_a, stretch=1)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("color: #2a2e39; border-left: 2px solid #2a2e39;")
        car_layout.addWidget(sep)

        # Lado Vuelta B
        box_b = QVBoxLayout()
        lbl_panel_b = QLabel("🏁 VUELTA B")
        lbl_panel_b.setStyleSheet("color: #ff9100; font-weight: bold; font-size: 13px;")
        lbl_panel_b.setAlignment(Qt.AlignmentFlag.AlignCenter)
        box_b.addWidget(lbl_panel_b)

        row_b = QHBoxLayout()
        self.map_b = TrackMapWidget(title="GPS Vuelta B")
        self.map_b.setMinimumHeight(240)
        self.gg_b = GGDiagramWidget()
        self.gg_b.setMinimumHeight(240)
        row_b.addWidget(self.map_b, stretch=1)
        row_b.addWidget(self.gg_b, stretch=1)
        box_b.addLayout(row_b, stretch=2)

        self.gauges_b = GaugesPanel()
        box_b.addWidget(self.gauges_b, stretch=1)
        car_layout.addLayout(box_b, stretch=1)

        self.bottom_scroll.setWidget(self.car_container)
        self.splitter.addWidget(self.bottom_scroll)

        self.splitter.setSizes([340, 460])
        root_layout.addWidget(self.splitter, stretch=1)

        # 3. Slider Inferior de Distancia
        slider_frame = QFrame()
        slider_frame.setStyleSheet("background-color: #111317; border-radius: 4px; padding: 4px;")
        slider_layout = QVBoxLayout(slider_frame)
        slider_layout.setContentsMargins(4, 2, 4, 2)

        self.lbl_dist_slider = QLabel("Distancia en Pista: 0.0 m")
        self.lbl_dist_slider.setStyleSheet("color: #8b949e; font-weight: bold; font-size: 11px;")
        self.lbl_dist_slider.setAlignment(Qt.AlignmentFlag.AlignCenter)
        slider_layout.addWidget(self.lbl_dist_slider)

        self.dist_slider = QSlider(Qt.Orientation.Horizontal)
        self.dist_slider.setStyleSheet("""
            QSlider::groove:horizontal { height: 8px; background: #263238; border-radius: 4px; }
            QSlider::sub-page:horizontal { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00e5ff, stop:1 #ff9100); border-radius: 4px; }
            QSlider::handle:horizontal { background: #ffffff; width: 16px; margin-top: -4px; margin-bottom: -4px; border-radius: 8px; }
        """)
        self.dist_slider.valueChanged.connect(self._on_slider_moved)
        slider_layout.addWidget(self.dist_slider)
        root_layout.addWidget(slider_frame)

        self._switch_mode_visibility()

    def _switch_mode_visibility(self):
        """Muestra u oculta subplots sin destruir los objetos."""
        mode = self.combo_mode.currentIndex()  # 0: Overlay, 1: Split
        if mode == 0:
            self.plot_overlay.show()
            self.plot_split_a.hide()
            self.plot_split_b.hide()
            if self.v_line not in self.plot_overlay.items:
                self.plot_overlay.addItem(self.v_line, ignoreBounds=True)
            if self.v_line in self.plot_split_a.items:
                self.plot_split_a.removeItem(self.v_line)
        else:
            self.plot_overlay.hide()
            self.plot_split_a.show()
            self.plot_split_b.show()
            if self.v_line in self.plot_overlay.items:
                self.plot_overlay.removeItem(self.v_line)
            if self.v_line not in self.plot_split_a.items:
                self.plot_split_a.addItem(self.v_line, ignoreBounds=True)

        self._update_plot_data()

    def set_data(self, full_df: pd.DataFrame, lap_summary: dict):
        self.full_df = full_df
        self.lap_summary = lap_summary

        self.combo_lap_a.blockSignals(True)
        self.combo_lap_b.blockSignals(True)
        self.combo_lap_a.clear()
        self.combo_lap_b.clear()

        if not lap_summary:
            return

        best_lap = min(lap_summary.keys(), key=lambda k: lap_summary[k]["time_s"])
        worst_lap = max(lap_summary.keys(), key=lambda k: lap_summary[k]["time_s"])

        for lap_num, info in lap_summary.items():
            t_s = info["time_s"]
            mins = int(t_s // 60)
            secs = t_s % 60
            tag = ""
            if lap_num == best_lap:
                tag = " 🟣 (Best Lap)"
            elif lap_num == worst_lap:
                tag = " 🔴 (Worst Lap)"

            label = f"Vuelta {lap_num} [{mins}:{secs:05.2f}]{tag}"
            self.combo_lap_a.addItem(label, userData=lap_num)
            self.combo_lap_b.addItem(label, userData=lap_num)

        self.combo_lap_a.blockSignals(False)
        self.combo_lap_b.blockSignals(False)

        self.combo_lap_a.setCurrentIndex(best_lap - 1)
        self.combo_lap_b.setCurrentIndex(worst_lap - 1)
        self._on_inputs_changed()

    def _on_inputs_changed(self):
        if self.full_df is None or self.full_df.empty:
            return

        lap_a_id = self.combo_lap_a.currentData()
        lap_b_id = self.combo_lap_b.currentData()
        if lap_a_id is None or lap_b_id is None:
            return

        self.df_a = self.full_df[self.full_df["lap_number"] == lap_a_id].copy()
        self.df_a["distance_m"] = self.df_a["distance_m"] - self.df_a["distance_m"].iloc[0]
        self.df_a["power_kw"] = (self.df_a["pack_voltage_v"] * self.df_a["pack_current_a"]) / 1000.0

        self.df_b = self.full_df[self.full_df["lap_number"] == lap_b_id].copy()
        self.df_b["distance_m"] = self.df_b["distance_m"] - self.df_b["distance_m"].iloc[0]
        self.df_b["power_kw"] = (self.df_b["pack_voltage_v"] * self.df_b["pack_current_a"]) / 1000.0

        dict_a = {
            "speed": self.df_a["speed_kmh"].values,
            "ax": self.df_a["ax_g"].values,
            "ay": self.df_a["ay_g"].values,
            "current": self.df_a["pack_current_a"].values,
            "voltage": self.df_a["pack_voltage_v"].values
        }
        dict_b = {
            "speed": self.df_b["speed_kmh"].values,
            "ax": self.df_b["ax_g"].values,
            "ay": self.df_b["ay_g"].values,
            "current": self.df_b["pack_current_a"].values,
            "voltage": self.df_b["pack_voltage_v"].values
        }

        self.map_a.load_track(self.df_a["lat"].values, self.df_a["lon"].values, dict_a)
        self.gg_a.load_data(self.df_a["ay_g"].values, self.df_a["ax_g"].values)

        self.map_b.load_track(self.df_b["lat"].values, self.df_b["lon"].values, dict_b)
        self.gg_b.load_data(self.df_b["ay_g"].values, self.df_b["ax_g"].values)

        time_a = self.lap_summary[lap_a_id]["time_s"]
        time_b = self.lap_summary[lap_b_id]["time_s"]
        diff = time_b - time_a
        color = "#00e676" if diff >= 0 else "#ff1744"
        sign = "+" if diff >= 0 else ""
        self.lbl_delta_summary.setText(f"Diferencia: <font color='{color}'>{sign}{diff:.2f} s</font>")

        self.max_common_dist = min(self.df_a["distance_m"].max(), self.df_b["distance_m"].max())
        self.dist_slider.setRange(0, int(self.max_common_dist))
        self.dist_slider.setValue(0)
        self.v_line.setBounds((0, self.max_common_dist))
        self.v_line.setValue(0)

        self._update_plot_data()
        self._sync_panels(0)

    def _update_plot_data(self):
        if self.df_a is None or self.df_b is None:
            return

        name_a = self.combo_chan_a.currentText()
        name_b = self.combo_chan_b.currentText()
        col_a, unit_a, short_a = CHANNELS[name_a]
        col_b, unit_b, short_b = CHANNELS[name_b]

        lap_a_num = self.combo_lap_a.currentData()
        lap_b_num = self.combo_lap_b.currentData()

        xa = self.df_a["distance_m"].values
        ya = self.df_a[col_a].values

        xb = self.df_b["distance_m"].values
        yb = self.df_b[col_b].values

        # 1. Cargar datos en curvas Overlay
        self.curve_overlay_a.setData(xa, ya)
        self.curve_overlay_b.setData(xb, yb)

        # 2. Cargar datos en curvas Split
        self.curve_split_a.setData(xa, ya)
        self.curve_split_b.setData(xb, yb)

        # Actualizar etiquetas de ejes
        if short_a == short_b and unit_a == unit_b:
            self.plot_overlay.setLabel('left', short_a, units=unit_a)
        else:
            self.plot_overlay.setLabel('left', f"{short_a} ({unit_a}) / {short_b} ({unit_b})")

        self.plot_split_a.setLabel('left', f"A: {short_a}", units=unit_a)
        self.plot_split_b.setLabel('left', f"B: {short_b}", units=unit_b)

        # Actualizar cuadro de leyenda
        self.legend_overlay.clear()
        self.legend_overlay.addItem(self.curve_overlay_a, f"V{lap_a_num}: {short_a}")
        self.legend_overlay.addItem(self.curve_overlay_b, f"V{lap_b_num}: {short_b}")

        # 3. Delta Time continuo
        ta = (self.df_a["timestamp_ms"].values - self.df_a["timestamp_ms"].iloc[0]) / 1000.0
        tb = (self.df_b["timestamp_ms"].values - self.df_b["timestamp_ms"].iloc[0]) / 1000.0
        common_x = np.linspace(0, self.max_common_dist, 500)
        delta = np.interp(common_x, xb, tb) - np.interp(common_x, xa, ta)
        self.curve_delta.setData(common_x, delta)
        self.zero_line.setData([0, self.max_common_dist], [0, 0])

        # 4. Establecer límites de visualización explícitos (Fix definitivo)
        self.refresh_view_ranges()

    def refresh_view_ranges(self):
        """Asegura que los rangos de cámara abarquen toda la vuelta (0 a 550m)."""
        if self.df_a is None or self.df_b is None:
            return

        col_a = CHANNELS[self.combo_chan_a.currentText()][0]
        col_b = CHANNELS[self.combo_chan_b.currentText()][0]
        ya = self.df_a[col_a].values
        yb = self.df_b[col_b].values

        x_max = max(self.max_common_dist, 550.0)
        y_min = float(min(np.min(ya), np.min(yb)))
        y_max = float(max(np.max(ya), np.max(yb)))
        pad = max((y_max - y_min) * 0.1, 1.0)

        # Fijar rangos
        self.plot_overlay.setXRange(0, x_max, padding=0.02)
        self.plot_overlay.setYRange(y_min - pad, y_max + pad, padding=0.02)

        self.plot_split_a.setXRange(0, x_max, padding=0.02)
        pad_a = max((np.max(ya) - np.min(ya)) * 0.1, 1.0)
        self.plot_split_a.setYRange(np.min(ya) - pad_a, np.max(ya) + pad_a, padding=0.02)

        self.plot_split_b.setXRange(0, x_max, padding=0.02)
        pad_b = max((np.max(yb) - np.min(yb)) * 0.1, 1.0)
        self.plot_split_b.setYRange(np.min(yb) - pad_b, np.max(yb) + pad_b, padding=0.02)

        self.plot_delta.setXRange(0, x_max, padding=0.02)
        self.plot_delta.enableAutoRange(axis=pg.ViewBox.YAxis)

    def _toggle_popup_graphs(self):
        if self.win_graphs is None or not self.win_graphs.isVisible():
            self.splitter.widget(0).setParent(None)
            self.win_graphs = DetachedWindow("BE KEY RACING — Gráficas Comparativas Multicanal", self)
            self.win_graphs.setCentralWidget(self.graph_view)
            self.win_graphs.closed.connect(self._re_dock_graphs)
            self.win_graphs.show()
            self.btn_pop_graphs.setText("🗗 Acoplar Gráficas")
            self.btn_pop_graphs.setStyleSheet("background-color: #b71c1c; color: white; font-weight: bold; padding: 4px 10px; border-radius: 4px; margin-left: 10px;")
            self.refresh_view_ranges()
        else:
            self.win_graphs.close()

    def _re_dock_graphs(self):
        self.splitter.insertWidget(0, self.graph_view)
        self.btn_pop_graphs.setText("🗗 Pop-up Gráficas")
        self.btn_pop_graphs.setStyleSheet("background-color: #004d40; color: #a7ffeb; font-weight: bold; padding: 4px 10px; border-radius: 4px; margin-left: 10px;")
        self.win_graphs = None
        self.refresh_view_ranges()

    def _toggle_popup_cars(self):
        if self.win_cars is None or not self.win_cars.isVisible():
            self.splitter.widget(1).setParent(None)
            self.win_cars = DetachedWindow("BE KEY RACING — Dinámicas & GPS Óscar Casillas (Lado a Lado)", self)
            self.win_cars.setCentralWidget(self.bottom_scroll)
            self.win_cars.closed.connect(self._re_dock_cars)
            self.win_cars.show()
            self.btn_pop_cars.setText("🗗 Acoplar GPS/Autos")
            self.btn_pop_cars.setStyleSheet("background-color: #b71c1c; color: white; font-weight: bold; padding: 4px 10px; border-radius: 4px; margin-left: 6px;")
        else:
            self.win_cars.close()

    def _re_dock_cars(self):
        self.splitter.insertWidget(1, self.bottom_scroll)
        self.btn_pop_cars.setText("🗗 Pop-up GPS/Autos")
        self.btn_pop_cars.setStyleSheet("background-color: #37474f; color: #00e5ff; font-weight: bold; padding: 4px 10px; border-radius: 4px; margin-left: 6px;")
        self.win_cars = None

    def _on_cursor_line_moved(self):
        val = self.v_line.value()
        self.dist_slider.blockSignals(True)
        self.dist_slider.setValue(int(val))
        self.dist_slider.blockSignals(False)
        self._sync_panels(val)

    def _on_slider_moved(self, dist_val):
        self.v_line.blockSignals(True)
        self.v_line.setValue(dist_val)
        self.v_line.blockSignals(False)
        self._sync_panels(dist_val)

    def _sync_panels(self, dist_val: float):
        if self.df_a is None or self.df_b is None:
            return

        self.lbl_dist_slider.setText(f"Distancia en Pista: {dist_val:.1f} m / {self.max_common_dist:.1f} m")

        idx_a = int(np.clip(np.searchsorted(self.df_a["distance_m"].values, dist_val), 0, len(self.df_a) - 1))
        idx_b = int(np.clip(np.searchsorted(self.df_b["distance_m"].values, dist_val), 0, len(self.df_b) - 1))

        self.map_a.update_cursor(idx_a)
        self.gg_a.update_cursor(idx_a)
        self.gauges_a.update_values(self.df_a.iloc[idx_a])

        self.map_b.update_cursor(idx_b)
        self.gg_b.update_cursor(idx_b)
        self.gauges_b.update_values(self.df_b.iloc[idx_b])