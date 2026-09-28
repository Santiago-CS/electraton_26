# Be Key ITESO Racing Team — Electrical & Embedded Systems

Repositorio central del área de electrónica, sistemas de baja tensión, telemetría y software embebido de **Be Key ITESO Racing Team**, escudería representativa del ITESO en el campeonato nacional **Electratón**.

Este espacio almacena el diseño de hardware (esquemáticos, layouts y archivos de manufactura Gerber/BOM), firmware de unidades de control, protocolos de comunicación en red vehicular e interfaces de adquisición de datos.

---

## 🏎️ Visión General de la Arquitectura

El sistema eléctrico del monoplaza opera bajo una estricta segregación galvánica y funcional entre dos dominios:

1. **Sistema de Alta Tensión (HV - High Voltage):**
   * Banco de celdas de tracción, interruptor general de aislamiento, fusibles de alta corriente y contactores principales.
   * Sistema de Gestión de Baterías (BMS) encargado del balanceo pasivo/activo, monitorización de temperatura por celda y corte seguro ante contingencias.
2. **Sistema de Baja Tensión y Control (LV - Low Voltage):**
   * Lógica de encendido de tracción condicionada a seguridades (*Interlocks*, paro de emergencia/kill switches y frenado hidráulico).
   * Red de comunicación de control y telemetría de parámetros en tiempo real hacia pits.
   * Procesamiento de datos de dinámica, ergonomía e instrumentación del habitáculo del piloto.

---

## 📁 Estructura del Repositorio

```text
├── docs/                   # Diagramas generales de cableado (harness), pinouts y notas técnicas
├── firmware/               # Proyectos de código fuente embebido (C / C++ / MicroPython)
│   ├── ecu_firmware/       # Lógica principal de tracción y máquina de estados del auto
│   └── wheel_firmware/     # Control del volante, display gráfico e interfaz de mando
├── hardware/               # Diseños electrónicos (Archivos de diseño CAD/EDA, BOMs, Gerbers)
│   ├── ecu/                # PCB de la Unidad de Control del Vehículo (ECU)
│   └── steering_wheel/     # PCB de mandos del volante ergonómico y display de telemetría
└── tools/                  # Scripts de procesamiento, telemetría y configuración en pista
```

---

## 🛠️ Subsistemas de Hardware (Hardware Directory)

### 1. `hardware/steering_wheel/`
* **Función:** Módulo de interfaz hombre-máquina (HMI) ubicado en el volante del monoplaza.
* **Componentes Clave:** 
  * Microcontrolador para adquisición de comandos del piloto (pulsadores, selectores rotativos y palancas de cambios/frenado regenerativo).
  * Pantalla gráfica/dashboard integrada para visualización de parámetros críticos en pista (voltaje total, corriente instantánea, velocidad y alarmas del BMS).
  * Comunicación inalámbrica o bus alámbrico hacia la ECU central.
* **Entregables en carpeta:**
  * Esquemáticos y PCB Layout.
  * Archivos Gerber de fabricación y ensamble (CPL/Pick and Place).
  * Lista de materiales (BOM) con números de parte y encapsulados.

### 2. `hardware/ecu/`
* **Función:** Unidad de Control Electrónico encargada de la lógica central del monoplaza y la máquina de estados de seguridad.
* **Componentes Clave:**
  * Microcontrolador principal de gestión en tiempo real.
  * Etapa de acondicionamiento de señales y lectura de sensores analógicos/digitales (posición del pedal de aceleración, presión de frenos, efecto Hall).
  * Drivers y relevadores para control de contactores de potencia y luces reglamentarias de estado (Tractive System Active Light).
  * Circuito de detección de fallas y enlace directo con la línea de paro de emergencia (*Shutdown Circuit*).
* **Entregables en carpeta:**
  * Diseño esquemático y diseño de pista multicapa (separación estricta de planos analógicos y digitales).
  * Archivos de manufactura listos para ensamble y fabricación de PCB.
  * Documentación de conectores y asignación de pines del arnés vehicular.

---

## 🔒 Protocolos de Seguridad y Trabajo en Taller

Todo trabajo desarrollado en este repositorio debe respetar de forma obligatoria las normas de seguridad del equipo:

1. **Aislamiento Eléctrico:** Antes de conectar cualquier tarjeta o programador al auto, verifique que la etapa de alta tensión se encuentre físicamente desconectada mediante el *Service Plug* y que el bus de potencia esté descargado a menos de 5 V.
2. **Uso de EPP:** Es mandatorio portar calzado de seguridad, bata de laboratorio y lentes normados durante cualquier sesión de soldadura, maquinado o pruebas dinámicas.
3. **Manejo de Baterías:** Queda prohibido intervenir módulos de acumuladores sin guantes dieléctricos y sin extintores adecuados (CO2 / Clase C / F-500) a la mano y despejados.

---

## 📜 Política de Confidencialidad y NDA

Este repositorio contiene información técnica propietaria, sensible y confidencial de **Be Key ITESO Racing Team**:

* Queda estrictamente prohibido clonar, transferir, duplicar, filtrar o exponer estos esquemáticos, diagramas o código a entidades ajenas al proyecto o a otras escuderías de competencia sin un **Convenio de Colaboración e Intercambio Técnico Oficial** firmado por el Team Principal, la Mesa Directiva y avalado por el responsable del PTI de ITESO.
* Cualquier divulgación deliberada o no autorizada causará la apertura inmediata de un expediente disciplinario, la expulsión definitiva del equipo y el escalamiento a las autoridades académicas y al Comité de Ética del ITESO.

---

## 👥 Contribuciones y Control de Versiones

1. **Ramas (`branches`):**
   * `main`: Código y versiones de PCB validadas y enviadas a manufactura. Protegida contra commits directos.
   * `hardware/feature-name` o `firmware/feature-name`: Ramas de desarrollo para nuevos esquemáticos, correcciones de pistas o módulos de software.
2. **Commits y Pull Requests:**
   * Utilice mensajes de commit descriptivos (ej. `fix(ecu): corregir pull-up en línea de interlock`).
   * Toda integración a `main` requiere la revisión y aprobación técnica del **Senior Chief Engineer** o del **Capitán de Área Técnica**.
