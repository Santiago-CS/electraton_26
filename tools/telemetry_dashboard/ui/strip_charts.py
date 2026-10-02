import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore import pyqtSignal

class StripChartsWidget(QWidget):
    cursor_moved = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.win = pg.GraphicsLayoutWidget()
        self.win.setBackground('#111317')
        layout.addWidget(self.win)
        
        # 1. Velocidad
        self.p_speed = self.win.addPlot(row=0, col=0)
        self.p_speed.setLabel('left', 'Vel', units='km/h')
        self.p_speed.showGrid(x=True, y=True, alpha=0.3)
        self.curve_speed = self.p_speed.plot(pen=pg.mkPen('#00e5ff', width=2))
        
        # 2. Pedales: Throttle y Brake
        self.p_pedals = self.win.addPlot(row=1, col=0)
        self.p_pedals.setLabel('left', 'Ctrl')
        self.p_pedals.showGrid(x=True, y=True, alpha=0.3)
        self.p_pedals.setXLink(self.p_speed)
        self.curve_throttle = self.p_pedals.plot(pen=pg.mkPen('#00e676', width=2), name="Throttle %")
        self.curve_brake = self.p_pedals.plot(pen=pg.mkPen('#ff1744', width=2), name="Brake bar")
        
        # 3. G-Force (Ax, Ay)
        self.p_accel = self.win.addPlot(row=2, col=0)
        self.p_accel.setLabel('left', 'G-Force', units='g')
        self.p_accel.showGrid(x=True, y=True, alpha=0.3)
        self.p_accel.setXLink(self.p_speed)
        self.curve_ax = self.p_accel.plot(pen=pg.mkPen('#ffea00', width=1.5), name="Ax (Long)")
        self.curve_ay = self.p_accel.plot(pen=pg.mkPen('#e040fb', width=1.5), name="Ay (Lat)")
        
        # 4. Térmicas / Batería
        self.p_can = self.win.addPlot(row=3, col=0)
        self.p_can.setLabel('left', 'Temp', units='°C')
        self.p_can.setLabel('bottom', 'Distancia Recorrida', units='m')
        self.p_can.showGrid(x=True, y=True, alpha=0.3)
        self.p_can.setXLink(self.p_speed)
        self.curve_mot_temp = self.p_can.plot(pen=pg.mkPen('#ff9100', width=2), name="Motor °C")
        self.curve_bat_temp = self.p_can.plot(pen=pg.mkPen('#00b0ff', width=2), name="Batt °C")
        
        # Cursor Sincronizado
        self.v_line = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen('#ffffff', width=2, style=pg.QtCore.Qt.PenStyle.DashLine))
        self.p_speed.addItem(self.v_line, ignoreBounds=True)
        self.v_line.sigPositionChanged.connect(self._on_line_moved)
        
        self.win.scene().sigMouseClicked.connect(self._on_mouse_clicked)
        self.x_data = None
        self.flag_markers = []

    def load_data(self, df):
        if df is None or df.empty:
            return
            
        self.x_data = df["distance_m"].values
        self.curve_speed.setData(self.x_data, df["speed_kmh"].values)
        self.curve_throttle.setData(self.x_data, df["throttle_pct"].values)
        self.curve_brake.setData(self.x_data, df["brake_bar"].values)
        self.curve_ax.setData(self.x_data, df["ax_g"].values)
        self.curve_ay.setData(self.x_data, df["ay_g"].values)
        self.curve_mot_temp.setData(self.x_data, df["motor_temp_c"].values)
        self.curve_bat_temp.setData(self.x_data, df["battery_temp_c"].values)
        
        # Autoescalar ejes
        self.p_speed.enableAutoRange()
        self.p_pedals.enableAutoRange()
        self.p_accel.enableAutoRange()
        self.p_can.enableAutoRange()
        
        if len(self.x_data) > 0:
            self.v_line.setBounds((self.x_data[0], self.x_data[-1]))
            self.v_line.setValue(self.x_data[0])

    def _on_mouse_clicked(self, event):
        pos = event.scenePos()
        for plot in [self.p_speed, self.p_pedals, self.p_accel, self.p_can]:
            if plot.sceneBoundingRect().contains(pos):
                mp = plot.vb.mapSceneToView(pos)
                self.set_cursor_by_x(mp.x())
                break

    def _on_line_moved(self):
        if self.x_data is None or len(self.x_data) == 0:
            return
        x_val = self.v_line.value()
        idx = int(np.clip(np.searchsorted(self.x_data, x_val), 0, len(self.x_data) - 1))
        self.cursor_moved.emit(idx)

    def set_cursor_index(self, index: int):
        if self.x_data is not None and 0 <= index < len(self.x_data):
            self.v_line.blockSignals(True)
            self.v_line.setValue(self.x_data[index])
            self.v_line.blockSignals(False)

    def set_cursor_by_x(self, x_val: float):
        if self.x_data is None or len(self.x_data) == 0:
            return
        idx = int(np.clip(np.searchsorted(self.x_data, x_val), 0, len(self.x_data) - 1))
        self.set_cursor_index(idx)
        self.cursor_moved.emit(idx)

    def add_custom_marker(self, label: str = "Flag"):
        if self.x_data is None:
            return
        curr_x = self.v_line.value()
        flag_line = pg.InfiniteLine(
            pos=curr_x, angle=90, pen=pg.mkPen('#ff007f', width=1.5, style=pg.QtCore.Qt.PenStyle.DotLine),
            label=label, labelOpts={'position': 0.9, 'color': '#ff007f'}
        )
        self.p_speed.addItem(flag_line)
        self.flag_markers.append(flag_line)