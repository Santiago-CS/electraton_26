import os
import pandas as pd
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QFileDialog,
    QPushButton, QLabel, QSlider, QToolBar, QComboBox, QTabWidget,
    QInputDialog
)
from PyQt6.QtCore import Qt

from tools.telemetry_dashboard.ui.track_map import TrackMapWidget
from tools.telemetry_dashboard.ui.gg_diagram import GGDiagramWidget
from tools.telemetry_dashboard.ui.strip_charts import StripChartsWidget
from tools.telemetry_dashboard.ui.gauges_panel import GaugesPanel
from tools.telemetry_dashboard.ui.settings_tab import SettingsTab
from tools.telemetry_dashboard.ui.comparison_tab import ComparisonTab
from tools.telemetry_dashboard.core.mock_source import generate_mock_session_csv
from tools.telemetry_dashboard.core.lap_analyzer import LapAnalyzer

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BE KEY RACING TEAM - Telemetry Suite | ITESO")
        self.resize(1650, 960)
        self.setStyleSheet("background-color: #0d0f12; color: #ffffff;")
        
        self.full_df = None
        self.active_df = None
        self.lap_summary = {}
        self._init_ui()

    def _init_ui(self):
        self.toolbar = QToolBar("Main Toolbar")
        self.toolbar.setStyleSheet("background-color: #1a1d24; border-bottom: 2px solid #00e5ff; padding: 6px;")
        self.addToolBar(self.toolbar)
        
        lbl_team = QLabel("⚡ BE KEY | ITESO RACING TEAM  ")
        lbl_team.setStyleSheet("font-weight: bold; font-size: 13px; color: #00e5ff; letter-spacing: 1px;")
        self.toolbar.addWidget(lbl_team)

        btn_open = QPushButton("📂 Abrir Stint (.csv)")
        btn_open.setStyleSheet("background-color: #2979ff; color: white; padding: 6px 14px; font-weight: bold; border-radius: 4px;")
        btn_open.clicked.connect(self._open_file_dialog)
        self.toolbar.addWidget(btn_open)

        btn_mock = QPushButton("🏁 Test Óscar Casillas (10 Vueltas)")
        btn_mock.setStyleSheet("background-color: #004d40; color: #a7ffeb; padding: 6px 12px; margin-left: 8px; font-weight: bold; border-radius: 4px;")
        btn_mock.clicked.connect(self._generate_and_load_mock)
        self.toolbar.addWidget(btn_mock)

        btn_marker = QPushButton("🚩 Poner Marca")
        btn_marker.setStyleSheet("background-color: #37474f; color: #ff80ab; padding: 6px 12px; margin-left: 8px; border-radius: 4px;")
        btn_marker.clicked.connect(self._add_marker_prompt)
        self.toolbar.addWidget(btn_marker)

        # Selector Único de Vuelta
        self.toolbar.addWidget(QLabel("   🏁 Vuelta:"))
        self.lap_combo = QComboBox()
        self.lap_combo.setStyleSheet("background-color: #111317; color: #00e5ff; font-weight: bold; padding: 4px 10px; border: 1px solid #00e5ff; border-radius: 4px; min-width: 150px;")
        self.lap_combo.currentIndexChanged.connect(self._on_lap_changed)
        self.toolbar.addWidget(self.lap_combo)

        self.lbl_lap_time = QLabel("")
        self.lbl_lap_time.setStyleSheet("font-weight: bold; color: #00e676; margin-left: 14px; font-size: 12px;")
        self.toolbar.addWidget(self.lbl_lap_time)

        # Contenedor de Pestañas
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabBar::tab { background: #1a1d24; color: #8b949e; padding: 8px 24px; font-weight: bold; border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 4px; }
            QTabBar::tab:selected { background: #00e5ff; color: #000; }
        """)
        self.setCentralWidget(self.tabs)

        # Pestaña 1: Análisis Tradicional (Vuelta Única)
        tab_analysis = QWidget()
        main_layout = QHBoxLayout(tab_analysis)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        left_layout = QVBoxLayout()
        self.strip_charts = StripChartsWidget()
        self.strip_charts.cursor_moved.connect(self._sync_cursor)
        left_layout.addWidget(self.strip_charts)
        
        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_slider.setStyleSheet("""
            QSlider::groove:horizontal { height: 6px; background: #263238; border-radius: 3px; }
            QSlider::sub-page:horizontal { background: #00e5ff; border-radius: 3px; }
            QSlider::handle:horizontal { background: #ffffff; width: 14px; margin-top: -4px; margin-bottom: -4px; border-radius: 7px; }
        """)
        self.time_slider.valueChanged.connect(self._on_slider_moved)
        left_layout.addWidget(self.time_slider)
        main_layout.addLayout(left_layout, stretch=6)
        
        right_layout = QVBoxLayout()
        top_right = QHBoxLayout()
        self.track_map = TrackMapWidget(title="GPS Track Map")
        self.gg_diagram = GGDiagramWidget()
        top_right.addWidget(self.track_map, stretch=1)
        top_right.addWidget(self.gg_diagram, stretch=1)
        right_layout.addLayout(top_right, stretch=1)
        
        self.gauges_panel = GaugesPanel()
        right_layout.addWidget(self.gauges_panel, stretch=1)
        main_layout.addLayout(right_layout, stretch=4)

        # Pestaña 2: Comparación Lado a Lado
        self.comparison_tab = ComparisonTab()

        # Pestaña 3: Configuración
        self.settings_tab = SettingsTab()

        self.tabs.addTab(tab_analysis, "📊 Análisis de Telemetría")
        self.tabs.addTab(self.comparison_tab, "⚖️ Comparación y Análisis Multicanal")
        #self.tabs.addTab(self.settings_tab, "⚙️ Configuración")

    def _open_file_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Abrir Log de Telemetría", "", "Archivos CSV (*.csv)")
        if path:
            self._process_and_load(pd.read_csv(path))

    def _generate_and_load_mock(self):
        mock_path = "casillas_10_laps.csv"
        generate_mock_session_csv(mock_path, laps=10)
        self._process_and_load(pd.read_csv(mock_path))

    def _add_marker_prompt(self):
        text, ok = QInputDialog.getText(self, "Añadir Marca", "Etiqueta para el punto actual (ej. 'Vértice C3', 'Frenada'):")
        if ok and text:
            self.strip_charts.add_custom_marker(text)

    def _process_and_load(self, df: pd.DataFrame):
        self.full_df, self.lap_summary = LapAnalyzer.segment_laps(df)
        
        self.lap_combo.blockSignals(True)
        self.lap_combo.clear()
        self.lap_combo.addItem("🌐 Toda la Sesión", userData=0)
        
        best_lap = min(self.lap_summary.keys(), key=lambda k: self.lap_summary[k]["time_s"])
        worst_lap = max(self.lap_summary.keys(), key=lambda k: self.lap_summary[k]["time_s"])
        
        for lap_num, info in self.lap_summary.items():
            t_s = info["time_s"]
            mins = int(t_s // 60)
            secs = t_s % 60
            tag = ""
            if lap_num == best_lap:
                tag = " 🟣 (Best Lap)"
            elif lap_num == worst_lap:
                tag = " 🔴 (Worst Lap)"

            label = f"Vuelta {lap_num} [{mins}:{secs:05.2f}]{tag}"
            self.lap_combo.addItem(label, userData=lap_num)
            
        self.lap_combo.blockSignals(False)

        # Cargar los datos en la pestaña de comparación
        self.comparison_tab.set_data(self.full_df, self.lap_summary)

        # Activar la Vuelta 1 en la Pestaña Principal y renderizar
        self.lap_combo.setCurrentIndex(1)
        self._load_lap_by_id(1)

    def _on_lap_changed(self, index):
        lap_id = self.lap_combo.currentData()
        if lap_id is not None:
            self._load_lap_by_id(lap_id)

    def _load_lap_by_id(self, lap_id: int):
        if self.full_df is None or self.full_df.empty:
            return

        if lap_id == 0:
            self.active_df = self.full_df.copy()
            total_m = (self.active_df["timestamp_ms"].iloc[-1] - self.active_df["timestamp_ms"].iloc[0]) / 60000.0
            self.lbl_lap_time.setText(f"Duración: {total_m:.1f} min | 10 Vueltas")
        else:
            self.active_df = self.full_df[self.full_df["lap_number"] == lap_id].copy()
            self.active_df["distance_m"] = self.active_df["distance_m"] - self.active_df["distance_m"].iloc[0]
            info = self.lap_summary.get(lap_id, {})
            if info:
                self.lbl_lap_time.setText(
                    f"T: {info['time_s']:.2f}s | Vmax: {info['max_speed']:.1f} km/h | Vmin: {info['min_speed']:.1f} km/h | Energía: {info['energy_wh']:.1f} Wh"
                )

        # Pintar datos inmediatamente
        self.strip_charts.load_data(self.active_df)
        
        dict_data = {
            "speed": self.active_df["speed_kmh"].values,
            "ax": self.active_df["ax_g"].values,
            "ay": self.active_df["ay_g"].values,
            "current": self.active_df["pack_current_a"].values,
            "voltage": self.active_df["pack_voltage_v"].values
        }
        self.track_map.load_track(
            self.active_df["lat"].values, self.active_df["lon"].values, dict_data
        )
        self.gg_diagram.load_data(self.active_df["ay_g"].values, self.active_df["ax_g"].values)
        
        self.time_slider.setRange(0, len(self.active_df) - 1)
        self.time_slider.setValue(0)
        self._sync_cursor(0)

    def _sync_cursor(self, index: int):
        if self.active_df is None or index >= len(self.active_df):
            return
        self.track_map.update_cursor(index)
        self.gg_diagram.update_cursor(index)
        self.gauges_panel.update_values(self.active_df.iloc[index])
        
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(index)
        self.time_slider.blockSignals(False)

    def _on_slider_moved(self, value: int):
        self.strip_charts.set_cursor_index(value)
        self._sync_cursor(value)