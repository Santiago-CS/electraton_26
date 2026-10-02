import numpy as np
import pandas as pd

class LapAnalyzer:
    GATE_RADIUS_M = 9.0
    MIN_LAP_TIME_S = 20.0

    @staticmethod
    def haversine_distance_m(lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1 = np.radians(lat1)
        phi2 = np.radians(lat2)
        dphi = np.radians(lat2 - lat1)
        dlambda = np.radians(lon2 - lon1)
        a = np.sin(dphi / 2.0)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2.0)**2
        return 2 * R * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    @classmethod
    def segment_laps(cls, df: pd.DataFrame, finish_lat=None, finish_lon=None):
        if df.empty:
            return df, {}

        lap_summary = {}

        if "lap_number" in df.columns:
            for lap_id, group in df.groupby("lap_number"):
                dt_s = (group["timestamp_ms"].iloc[-1] - group["timestamp_ms"].iloc[0]) / 1000.0
                dist = group["distance_m"].iloc[-1] - group["distance_m"].iloc[0]
                
                if dt_s < 5.0 or dist < 50.0:
                    continue

                dt_step = np.diff(group["timestamp_ms"].values, prepend=group["timestamp_ms"].iloc[0]) / 1000.0
                power_w = group["pack_voltage_v"].values * group["pack_current_a"].values
                energy_wh = np.sum(power_w * dt_step) / 3600.0

                lap_summary[int(lap_id)] = {
                    "time_s": dt_s,
                    "dist_m": dist,
                    "avg_speed": group["speed_kmh"].mean(),
                    "max_speed": group["speed_kmh"].max(),
                    "min_speed": group["speed_kmh"].min(),
                    "energy_wh": energy_wh,
                    "max_motor_temp": group["motor_temp_c"].max()
                }
            return df, lap_summary

        if finish_lat is None or finish_lon is None:
            finish_lat = df["lat"].iloc[0]
            finish_lon = df["lon"].iloc[0]

        dists = cls.haversine_distance_m(df["lat"].values, df["lon"].values, finish_lat, finish_lon)
        timestamps = df["timestamp_ms"].values / 1000.0
        n = len(df)
        
        lap_indices = [0]
        last_t = timestamps[0]
        i = 1
        while i < n:
            if dists[i] <= cls.GATE_RADIUS_M:
                if (timestamps[i] - last_t) >= cls.MIN_LAP_TIME_S:
                    w_end = min(i + 100, n)
                    min_idx = i + int(np.argmin(dists[i:w_end]))
                    lap_indices.append(min_idx)
                    last_t = timestamps[min_idx]
                    i = min_idx + int(cls.MIN_LAP_TIME_S * 50)
                    continue
            i += 1
            
        lap_indices.append(n)
        lap_col = np.ones(n, dtype=int)

        actual_lap = 1
        for k in range(1, len(lap_indices)):
            s_idx = lap_indices[k - 1]
            e_idx = lap_indices[k]
            sub = df.iloc[s_idx:e_idx]
            dt_s = (sub["timestamp_ms"].iloc[-1] - sub["timestamp_ms"].iloc[0]) / 1000.0
            dist = sub["distance_m"].iloc[-1] - sub["distance_m"].iloc[0]
            
            if dt_s < 5.0 or dist < 50.0:
                continue

            dt_step = np.diff(sub["timestamp_ms"].values, prepend=sub["timestamp_ms"].iloc[0]) / 1000.0
            energy_wh = np.sum(sub["pack_voltage_v"].values * sub["pack_current_a"].values * dt_step) / 3600.0

            lap_col[s_idx:e_idx] = actual_lap
            lap_summary[actual_lap] = {
                "time_s": dt_s,
                "dist_m": dist,
                "avg_speed": sub["speed_kmh"].mean(),
                "max_speed": sub["speed_kmh"].max(),
                "min_speed": sub["speed_kmh"].min(),
                "energy_wh": energy_wh,
                "max_motor_temp": sub["motor_temp_c"].max()
            }
            actual_lap += 1

        df["lap_number"] = lap_col
        return df, lap_summary