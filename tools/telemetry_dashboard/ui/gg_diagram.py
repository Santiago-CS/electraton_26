import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout

class GGDiagramWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.plot_widget = pg.PlotWidget(title="G-G Diagram (Círculo de Tracción)")
        self.plot_widget.setBackground('#111317')
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setLabel('bottom', "Aceleración Lateral (Ay)", units="g")
        self.plot_widget.setLabel('left', "Aceleración Longitudinal (Ax)", units="g")
        self.plot_widget.setXRange(-1.6, 1.6)
        self.plot_widget.setYRange(-1.6, 1.6)
        layout.addWidget(self.plot_widget)
        
        # Círculos guía de 0.5g, 1.0g y 1.5g
        angles = np.linspace(0, 2 * np.pi, 120)
        for g in [0.5, 1.0, 1.5]:
            pen = pg.mkPen((80, 80, 80, 120), width=1, style=pg.QtCore.Qt.PenStyle.DashLine)
            self.plot_widget.plot(g * np.cos(angles), g * np.sin(angles), pen=pen)
        
        # Ejes centrales 0
        self.plot_widget.plot([-1.6, 1.6], [0, 0], pen=pg.mkPen((100, 100, 100, 150), width=1))
        self.plot_widget.plot([0, 0], [-1.6, 1.6], pen=pg.mkPen((100, 100, 100, 150), width=1))
        
        # Nube histórica de puntos
        self.scatter_trail = pg.ScatterPlotItem(size=4, brush=pg.mkBrush(0, 180, 255, 60), pen=None)
        # Marcador del cursor actual
        self.current_marker = pg.ScatterPlotItem(size=14, brush=pg.mkBrush(255, 220, 0, 255), pen=pg.mkPen('w', width=2))
        
        self.plot_widget.addItem(self.scatter_trail)
        self.plot_widget.addItem(self.current_marker)
        
        self.ay = None
        self.ax = None

    def load_data(self, ay_g: np.ndarray, ax_g: np.ndarray):
        self.ay = ay_g
        self.ax = ax_g
        self.scatter_trail.setData(x=ay_g, y=ax_g)

    def update_cursor(self, index: int):
        if self.ay is not None and 0 <= index < len(self.ay):
            self.current_marker.setData([{'pos': (self.ay[index], self.ax[index])}])