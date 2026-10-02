import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QLabel
from PyQt6.QtGui import QColor

# Colormap divergente personalizado para Aceleración y Frenada:
# -1.5g (Frenada máxima) = Rojo (#ff1744)
#  0.0g (Neutro/Transición) = Gris/Blanco (#aaaaaa)
# +1.0g (Aceleración a fondo) = Verde Neón (#00e676)
POSITIONS_ACCEL = np.array([0.0, 0.5, 1.0])
COLORS_ACCEL = np.array([
    [255, 23, 68, 255],     # Rojo intenso (Frenada)
    [160, 160, 160, 255],   # Gris neutro
    [0, 230, 118, 255]      # Verde (Aceleración)
], dtype=np.ubyte)
ACCEL_COLORMAP = pg.ColorMap(pos=POSITIONS_ACCEL, color=COLORS_ACCEL)


class TrackMapWidget(QWidget):
    def __init__(self, title="GPS Track Map", parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        
        # Barra superior con selector de capa de calor (Heatmap Layer)
        top_bar = QHBoxLayout()
        lbl_mode = QLabel("Capa:")
        lbl_mode.setStyleSheet("color: #8b949e; font-size: 11px; font-weight: bold;")
        top_bar.addWidget(lbl_mode)

        self.combo_channel = QComboBox()
        self.combo_channel.setStyleSheet("""
            QComboBox {
                background-color: #1a1d24;
                color: #00e5ff;
                font-size: 11px;
                font-weight: bold;
                padding: 2px 6px;
                border: 1px solid #2a2e39;
                border-radius: 3px;
            }
            QComboBox QAbstractItemView { background-color: #1a1d24; color: white; }
        """)
        self.combo_channel.addItems([
            "Velocidad (km/h)",
            "Aceleración/Frenada (Ax)",
            "Fuerza Lateral (|Ay|)",
            "Corriente Batería (A)",
            "Potencia (kW)"
        ])
        self.combo_channel.currentIndexChanged.connect(self._replot_heatmap)
        top_bar.addWidget(self.combo_channel)
        top_bar.addStretch()
        layout.addLayout(top_bar)
        
        # Plot de pyqtgraph
        self.plot_widget = pg.PlotWidget(title=title)
        self.plot_widget.setBackground('#111317')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.getAxis('bottom').setStyle(showValues=False)
        self.plot_widget.getAxis('left').setStyle(showValues=False)
        layout.addWidget(self.plot_widget)
        
        self.track_scatter = pg.ScatterPlotItem()
        self.cursor_marker = pg.ScatterPlotItem(
            size=14, pen=pg.mkPen('w', width=2), brush=pg.mkBrush(255, 30, 30, 255), symbol='o'
        )
        self.plot_widget.addItem(self.track_scatter)
        self.plot_widget.addItem(self.cursor_marker)
        
        self.x_coords = None
        self.y_coords = None
        self.raw_data_dict = {}

    def load_track(self, lat: np.ndarray, lon: np.ndarray, data_dict: dict):
        if len(lat) == 0:
            return
            
        lat_rad = np.radians(lat[0])
        self.x_coords = (lon - lon[0]) * 111139.0 * np.cos(lat_rad)
        self.y_coords = (lat - lat[0]) * 111139.0
        self.raw_data_dict = data_dict
        
        self._replot_heatmap()
        self.plot_widget.autoRange()

    def _replot_heatmap(self):
        if self.x_coords is None or not self.raw_data_dict:
            return

        choice = self.combo_channel.currentIndex()

        if choice == 0:  # Velocidad
            vals = self.raw_data_dict.get("speed", np.zeros_like(self.x_coords))
            min_v, max_v = np.min(vals), np.max(vals)
            norm_vals = np.clip((vals - min_v) / (max(max_v - min_v, 1e-3)), 0.0, 1.0)
            cmap = pg.colormap.get('turbo')
            colors = cmap.map(norm_vals, mode='qcolor')

        elif choice == 1:  # Aceleración / Frenada (Ax)
            vals = self.raw_data_dict.get("ax", np.zeros_like(self.x_coords))
            # Escala centrada en 0: de -1.5g a +1.0g
            # Normalizamos donde 0.5 = 0.0g
            norm_vals = np.clip((vals + 1.5) / 2.5, 0.0, 1.0)
            colors = ACCEL_COLORMAP.map(norm_vals, mode='qcolor')

        elif choice == 2:  # Lateral puro (|Ay|)
            vals = np.abs(self.raw_data_dict.get("ay", np.zeros_like(self.x_coords)))
            min_v, max_v = np.min(vals), np.max(vals)
            norm_vals = np.clip((vals - min_v) / (max(max_v - min_v, 1e-3)), 0.0, 1.0)
            cmap = pg.colormap.get('plasma')
            colors = cmap.map(norm_vals, mode='qcolor')

        elif choice == 3:  # Corriente
            vals = self.raw_data_dict.get("current", np.zeros_like(self.x_coords))
            min_v, max_v = np.min(vals), np.max(vals)
            norm_vals = np.clip((vals - min_v) / (max(max_v - min_v, 1e-3)), 0.0, 1.0)
            cmap = pg.colormap.get('inferno')
            colors = cmap.map(norm_vals, mode='qcolor')

        else:  # Potencia (kW)
            v = self.raw_data_dict.get("voltage", np.zeros_like(self.x_coords))
            i = self.raw_data_dict.get("current", np.zeros_like(self.x_coords))
            vals = (v * i) / 1000.0
            min_v, max_v = np.min(vals), np.max(vals)
            norm_vals = np.clip((vals - min_v) / (max(max_v - min_v, 1e-3)), 0.0, 1.0)
            cmap = pg.colormap.get('viridis')
            colors = cmap.map(norm_vals, mode='qcolor')

        points = [
            {'pos': (self.x_coords[i], self.y_coords[i]), 'size': 5, 'brush': colors[i], 'pen': None}
            for i in range(len(self.x_coords))
        ]
        self.track_scatter.setData(points)

    def update_cursor(self, index: int):
        if self.x_coords is not None and 0 <= index < len(self.x_coords):
            self.cursor_marker.setData([{'pos': (self.x_coords[index], self.y_coords[index])}])