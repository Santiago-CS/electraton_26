import numpy as np
import pandas as pd
from scipy.interpolate import splprep, splev

def get_casillas_track_points():
    """
    Waypoints clave digitalizados del trazado real del Kartódromo Óscar Casillas
    (550 m, 9 curvas, sentido anti-horario).
    Coordenada GPS base: 20.736650 N, -103.484050 W (Zapopan, Jal.)
    """
    # Coordenadas relativas en metros aproximadas siguiendo el diagrama antihorario:
    # Inicio: Recta de meta diagonal inferior (flechas blancas) hacia la izquierda
    # -> Curva 1: 90° a la derecha subiendo
    # -> Curva 2 y 3: Subida vertical y curva superior hacia la derecha
    # -> Curva 4 y 5: Horquilla superior derecha
    # -> Curva 6 y 7: Chicana central en S
    # -> Curva 8: Horquilla este
    # -> Curva 9: Última curva hacia recta de meta
    waypoints = np.array([
        [-15.0, -32.0],  # Meta (Start/Finish)
        [-40.0, -32.0],  # Fin de recta de meta
        [-48.0, -20.0],  # Curva 1 (hacia arriba)
        [-48.0,  15.0],  # Recta oeste
        [-45.0,  40.0],  # Curva 2
        [-20.0,  48.0],  # Curva 3 (entrada a recta norte)
        [ 20.0,  48.0],  # Recta norte
        [ 45.0,  45.0],  # Curva 4
        [ 52.0,  30.0],  # Horquilla superior este (Curva 5)
        [ 30.0,  22.0],  # Regreso por recta intermedia
        [  0.0,  20.0],  # Entrada chicana
        [-15.0,   8.0],  # Curva 6 (chicana)
        [-22.0, -10.0],  # Curva 7 (chicana)
        [-05.0, -15.0],  # Salida chicana hacia horquilla este
        [ 32.0, -10.0],  # Recta previa horquilla sur
        [ 50.0, -20.0],  # Curva 8 (Horquilla este)
        [ 38.0, -32.0],  # Salida horquilla sur
        [ 10.0, -32.0],  # Curva 9 (recta de meta)
        [-15.0, -32.0]   # Cierre en meta
    ])
    
    # Interpolar con Spline cúbico cerrado para suavizar el circuito
    tck, u = splprep([waypoints[:, 0], waypoints[:, 1]], s=2.0, per=True)
    u_new = np.linspace(0, 1, 600, endpoint=False)
    x_m, y_m = splev(u_new, tck)
    
    # Escalar a exactamente 550 metros
    dx = np.gradient(x_m)
    dy = np.gradient(y_m)
    current_len = np.sum(np.sqrt(dx**2 + dy**2))
    scale = 550.0 / current_len
    x_m *= scale
    y_m *= scale
    
    # Convertir a coordenadas GPS WGS84
    base_lat = 20.736650
    base_lon = -103.484050
    lat = base_lat + (y_m / 111139.0)
    lon = base_lon + (x_m / (111139.0 * np.cos(np.radians(base_lat))))
    
    return x_m, y_m, lat, lon

def generate_mock_session_csv(filepath: str, laps: int = 10, freq_hz: int = 50):
    dt = 1.0 / freq_hz
    x_base, y_base, lat_base, lon_base = get_casillas_track_points()
    n_track = len(x_base)

    # Duración realista por vuelta para 550m en monoplaza eléctrico (~37 a 41 segundos)
    # Vuelta 4 y 7 con mejor ritmo (Best Lap = 37.8s)
    lap_durations = [40.5, 39.8, 38.6, 37.8, 38.2, 38.9, 37.9, 39.2, 40.0, 40.8]
    if laps > len(lap_durations):
        lap_durations += [39.0] * (laps - len(lap_durations))
    lap_durations = lap_durations[:laps]

    all_dfs = []
    current_time_ms = 0.0
    accum_dist_m = 0.0

    for lap_idx, lap_time in enumerate(lap_durations):
        n_samples = int(lap_time * freq_hz)
        sample_indices = np.linspace(0, n_track - 1, n_samples)
        
        x = np.interp(sample_indices, np.arange(n_track), x_base)
        y = np.interp(sample_indices, np.arange(n_track), y_base)
        lat = np.interp(sample_indices, np.arange(n_track), lat_base)
        lon = np.interp(sample_indices, np.arange(n_track), lon_base)
        
        # Curvatura
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curv = np.abs(dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2)**1.5, 1e-5)
        
        # Velocidad coherente con agarre físico (máx ~78 km/h en recta norte, mín ~26 km/h en horquillas)
        pace = 38.0 / lap_time
        target_v = np.clip((78.0 / (1.0 + 35.0 * curv)) * pace, 24.0, 78.0)
        
        # Suavizado de aceleración (evitar ruido de alta frecuencia en la gráfica)
        speed_kmh = pd.Series(target_v).rolling(window=15, min_periods=1, center=True).mean().values
        
        v_ms = speed_kmh / 3.6
        ax_ms2 = np.gradient(v_ms, dt)
        ax_g = np.clip(ax_ms2 / 9.81, -1.35, 0.85)
        
        heading = np.arctan2(dy, dx)
        yaw_rate = np.degrees(np.gradient(heading, dt))
        ay_g = np.clip((v_ms * np.radians(yaw_rate)) / 9.81, -1.45, 1.45)
        
        motor_rpm = speed_kmh * 65.0
        throttle = np.clip(ax_g * 120.0, 0.0, 100.0)
        brake = np.clip(-ax_g * 35.0, 0.0, 45.0)
        
        # Evolución térmica y de voltaje progresiva
        total_s = current_time_ms / 1000.0
        motor_temp = 30.0 + (total_s * 0.04) + (throttle * 0.06)
        batt_temp = 26.0 + (total_s * 0.02)
        pack_v = 52.8 - (total_s * 0.007) - (throttle * 0.025)
        pack_i = np.clip(throttle * 1.5 - (brake * 0.25), 0.0, 145.0)
        
        t_arr = np.linspace(0, lap_time * 1000.0, n_samples) + current_time_ms
        dist_lap = np.cumsum(v_ms * dt)
        
        df_lap = pd.DataFrame({
            "timestamp_ms": t_arr.astype(int),
            "distance_m": np.round(dist_lap + accum_dist_m, 2),
            "lat": lat,
            "lon": lon,
            "speed_kmh": np.round(speed_kmh, 2),
            "ax_g": np.round(ax_g, 3),
            "ay_g": np.round(ay_g, 3),
            "yaw_rate_dps": np.round(yaw_rate, 2),
            "motor_rpm": np.round(motor_rpm, 1),
            "throttle_pct": np.round(throttle, 1),
            "brake_bar": np.round(brake, 1),
            "motor_temp_c": np.round(motor_temp, 2),
            "battery_temp_c": np.round(batt_temp, 2),
            "pack_voltage_v": np.round(pack_v, 2),
            "pack_current_a": np.round(pack_i, 2),
            "lap_number": lap_idx + 1
        })
        
        current_time_ms += lap_time * 1000.0
        accum_dist_m += dist_lap[-1]
        all_dfs.append(df_lap)
        
    full_df = pd.concat(all_dfs, ignore_index=True)
    full_df.to_csv(filepath, index=False)
    print(f"[Mock] Sesión Óscar Casillas ({laps} vueltas, 550m) guardada en: {filepath}")

if __name__ == "__main__":
    generate_mock_session_csv("casillas_10_laps.csv", laps=10)