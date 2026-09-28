from dataclasses import dataclass, field
import time

@dataclass
class TelemetryFrame:
    # Sincronización
    timestamp_ms: float = field(default_factory=lambda: time.time() * 1000.0)

    # GPS (ATGM336H / BZGNSS M10 @ 10 Hz)
    lat: float = 0.0
    lon: float = 0.0
    gps_speed_kmh: float = 0.0
    satellites: int = 0

    # Dinámica Chasis - IMU BNO085 (Aceleraciones lineales y giros)
    ax_g: float = 0.0      # Longitudinal (+ Aceleración / - Frenada)
    ay_g: float = 0.0      # Lateral (+ Curva izquierda / - Curva derecha)
    az_g: float = 0.0      # Vertical
    yaw_rate_dps: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0

    # Tren Motriz & Red CAN (Inversor / BMS / Ruedas)
    wheel_speed_kmh: float = 0.0
    motor_rpm: float = 0.0
    motor_temp_c: float = 0.0
    inverter_temp_c: float = 0.0
    pack_voltage_v: float = 0.0
    pack_current_a: float = 0.0
    pack_soc_pct: float = 0.0
    pack_temp_max_c: float = 0.0
    throttle_pct: float = 0.0
    brake_pressure_bar: float = 0.0