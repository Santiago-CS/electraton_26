# ⚡ BE KEY RACING TEAM — Telemetry Suite & Data Analysis

Suite modular de telemetría de alto rendimiento desarrollada en **Python**, **PyQt6** y **pyqtgraph** para el análisis dinámico, térmico y de consumo energético del monoplaza eléctrico de **Be Key ITESO Racing Team** (División Electratón).

Diseñada para operar en el box de pits tanto en análisis offline (logs `.csv` capturados por la tarjeta de adquisición a bordo **NXP FRDM-i.MX93**) como en validación de algoritmos de vuelta en el circuito del **Kartódromo Óscar Casillas** (550 m).

---

## 🏎️ Características Principales

### 1. 📊 Análisis de Telemetría (Vuelta Única)
* **Visualización de Canales Primarios:**
  * Curva continua de Velocidad ($0 - 80\text{ km/h}$).
  * Controles del piloto: Porcentaje de acelerador (Throttle %) y presión de circuito hidráulico de freno (Brake bar).
  * Dinámica vehicular en tiempo real: Aceleración longitudinal ($A_x$) y aceleración lateral ($A_y$).
  * Gestión térmica: Temperaturas en tiempo real del motor eléctrico y del pack de acumuladores ($^\circ\text{C}$).
* **Sincronización Total con Cursor:** Desplazamiento simultáneo del marcador en mapas GPS, círculos de tracción G-G y tablero digital.
* **Marcadores en Pista (Flag Markers):** Inserción interactiva de marcas puntuales sobre la gráfica de distancia para identificar vértices de curva (Apex), puntos de frenada o incidencias en pista.

### 2. ⚖️ Comparación y Análisis Multicanal (Lado a Lado)
* **Detección Automática de Vueltas Clave:** El analizador segmenta la sesión e identifica automáticamente la **🟣 Mejor Vuelta (Best Lap)** y la **🔴 Peor Vuelta (Worst Lap)**.
* **Modos de Visualización:**
  * **Superpuestas (Overlay):** Ambas trazas graficadas en un mismo eje con caja de leyenda dinámica (*Legend Item*) para comparar diferencias de trazado y velocidad punta.
  * **Separadas (Split):** Subplots independientes sincronizados por distancia con enlaces de eje X vinculados (*linked axes*).
* **Canal de $\Delta\text{ Time}$ Continuo:** Gráfica acumulada de ganancia/pérdida de tiempo en segundos a lo largo de cada metro de la pista.
* **Comparación Cruzada:** Capacidad de comparar cualquier combinación de canales (ej. *Velocidad vs Consumo de Corriente*, o *Vuelta 4 vs Vuelta 10*).
* **Doble Inspector de Autos (Side-by-Side):**
  * Mapas gemelos con trazado completo del Kartódromo Óscar Casillas.
  * Diagramas G-G duales para contrastar aprovechamiento del agarre mecánico.
  * Tableros de telemetría independientes actualizados al mismo metro de distancia.
* **Pop-ups Desacoplables (Multi-Monitor):** Botones para desacoplar a ventanas flotantes independientes tanto el bloque de gráficas como el bloque de mapas GPS/dinámicas.

### 3. 🗺️ GPS Track Map & Mapas Térmicos Dinámicos
* Reconstrucción bidimensional del circuito mediante proyección conformal local (metros reales desde lat/lon WGS84).
* Selector de capa de calor (Heatmap Layer):
  * **Velocidad (km/h):** Mapa estándar de ritmo.
  * **Aceleración/Frenada ($A_x$):** Mapa divergente (Rojo intenso en frenada fuerte, Verde brillante en aceleración plena).
  * **Fuerza Lateral ($\vert{}A_y\vert{}$):** Detección de zonas de alta carga lateral en curvas.
  * **Corriente Batería ($A$):** Identificación visual de picos de consumo de corriente.
  * **Potencia ($kW$):** Trazado de potencia instantánea entregada por el inversor.

---

## 📁 Estructura del Módulo

```text
tools/telemetry_dashboard/
├── core/
│   ├── __init__.py
│   ├── lap_analyzer.py      # Detección de cortes de meta (Haversine), cálculo de Best/Worst lap y métricas
│   └── mock_source.py       # Generador de sesiones simuladas en el Kartódromo Óscar Casillas
├── ui/
│   ├── __init__.py
│   ├── comparison_tab.py    # Pestaña de comparación multicanal lado a lado con soporte pop-up
│   ├── gauges_panel.py      # Tablero numérico de telemetría instantánea
│   ├── gg_diagram.py        # Gráfica del círculo de fricción G-G (Ax vs Ay)
│   ├── main_window.py       # Ventana principal, barra de herramientas y gestión de pestañas
│   ├── settings_tab.py      # Configuración de estilos visuales, paletas de color e idioma
│   ├── strip_charts.py      # Trazador temporal multicanal por vuelta
│   └── track_map.py         # Mapa de pista GPS con capas térmicas personalizadas
├── assets/                  # Recursos gráficos e isotipos del equipo
│   └── logo.jpeg
├── main.py                  # Punto de entrada de la aplicación
├── requirements.txt         # Dependencias del entorno
└── README.md                # Documentación técnica del módulo
```

## 📦 Contrato de Datos (Formato CSV)

La herramienta procesa archivos `.csv` exportados por el sistema de adquisición a bordo del monoplaza (procesados en el Cortex-A55 / Cortex-M33 de la tarjeta i.MX93). El archivo debe contener los siguientes encabezados:

| Columna | Tipo | Unidad | Descripción |
| :--- | :---: | :---: | :--- |
| `timestamp_ms` | `int` | ms | Tiempo relativo desde el inicio del registro en milisegundos |
| `distance_m` | `float` | m | Distancia integrada acumulada en pista por odometría |
| `lat` | `float` | Grados | Latitud GPS en formato WGS84 decimal |
| `lon` | `float` | Grados | Longitud GPS en formato WGS84 decimal |
| `speed_kmh` | `float` | km/h | Velocidad lineal del monoplaza calculada |
| `ax_g` | `float` | g | Aceleración longitudinal (+ aceleración, - frenada) |
| `ay_g` | `float` | g | Aceleración lateral (+ curvas izquierda, - curvas derecha) |
| `yaw_rate_dps` | `float` | deg/s | Velocidad angular de guiñada del chasis |
| `motor_rpm` | `float` | RPM | Revoluciones por minuto del motor de tracción |
| `throttle_pct` | `float` | % | Apertura o porcentaje del pedal de acelerador (0 a 100) |
| `brake_bar` | `float` | bar | Presión en la línea del circuito hidráulico de freno |
| `motor_temp_c` | `float` | °C | Temperatura interna en carcasa/devanados del motor |
| `battery_temp_c` | `float` | °C | Temperatura máxima registrada en módulos de celdas |
| `pack_voltage_v` | `float` | V | Tensión total del banco de acumuladores de tracción |
| `pack_current_a` | `float` | A | Corriente demandada instantánea entregada al inversor |
| `lap_number` | `int` | - | Identificador de vuelta computado (1, 2, 3...) |

---

## 🚀 Instalación y Ejecución

### 1. Requisitos Previos
* **Python 3.10 o superior**.
* Entorno virtual de Python recomendado.

### 2. Instalación de Dependencias
Desde la raíz del repositorio (`electraton_26/`):

```bash
pip install -r tools/telemetry_dashboard/requirements.txt
```

### 3. Ejecución de la Suite
Ejecuta la suite como módulo de Python desde la raíz del proyecto:

```bash
python -m tools.telemetry_dashboard.main
```

### 4. Generación y Carga de Sesiones de Prueba (Mock Data)
Para probar todas las funciones sin conectar hardware físico:

* Dentro de la aplicación, haz clic en el botón superior "🏁 Test Óscar Casillas (10 Vueltas)".
* O genera el log directamente por terminal:

```bash
python -m tools.telemetry_dashboard.core.mock_source
```

>El archivo se guardará automáticamente en tools/telemetry_dashboard/casillas_10_laps.csv y podrá abrirse con el botón 📂 Abrir Stint (.csv).

## 👥 Flujo de Desarrollo y Control de Versiones

* **Rama base:** Todo desarrollo de utilerías debe partir e integrarse a la rama `tools/dev`.
* **Rama de trabajo:** Ramas de tarea bajo la convención `tools/<nombre-tarea>` (ej. `tools/telemetry-gui-dashboard`).
* **Pull Requests:** Los cambios deben dirigirse a `tools/dev` mediante PR convencional y requieren la validación técnica del **Senior Chief Engineer** o del **Capitán de Área Técnica**. Está prohibido realizar commits directos a `main`.
