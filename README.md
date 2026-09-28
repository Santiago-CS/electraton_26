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
├── FRDM-iMX93/
│   ├── 3D Printed Enclosure for FRDM-IMX93/
│   └── FRDM-IMX93-DESIGNFILES/
│       ├── 3D Module/
│       ├── Design Files/
│       ├── Gerbers/
│       └── PDF/
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

### 1. `FRDM-iMX93/`
* **Función:** Plataforma de cómputo embebido y adquisición de datos basada en el procesador NXP i.MX93, empleada para telemetría avanzada, procesamiento y soporte de computación a bordo.
* **Carpetas del módulo:**
  * `3D Printed Enclosure for FRDM-IMX93/`: Archivos de diseño y modelos del gabinete / case para manufactura aditiva y montaje en el chasis.
  * `FRDM-IMX93-DESIGNFILES/`: Archivos de diseño originales de la tarjeta de desarrollo (`3D Module/`, `Design Files/`, `Gerbers/`, `PDF/`).

### 2. `hardware/steering_wheel/`
* **Función:** Módulo de interfaz hombre-máquina (HMI) ubicado en el volante del monoplaza.
* **Componentes Clave:** 
  * Microcontrolador para adquisición de comandos del piloto (pulsadores, selectores rotativos y palancas de cambios/frenado regenerativo).
  * Pantalla gráfica/dashboard integrada para visualización de parámetros críticos en pista (voltaje total, corriente instantánea, velocidad y alarmas del BMS).
  * Comunicación inalámbrica o bus alámbrico hacia la ECU central.
* **Entregables en carpeta:**
  * Esquemáticos y PCB Layout.
  * Archivos Gerber de fabricación y ensamble (CPL/Pick and Place).
  * Lista de materiales (BOM) con números de parte y encapsulados.

### 3. `hardware/ecu/`
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

## 🌿 Flujo de Trabajo con Git y Creación de Pull Requests (PR)

Un **Pull Request (PR)** es una solicitud formal para que los líderes del área técnica revisen, validen y aprueben tus cambios antes de mezclarlos con el trabajo de los demás. **Nadie sube cambios directos a la rama `main`.**

### 1. Estructura de Ramas Base
* `main`: Producción oficial. Exclusiva para diseños probados y versiones finales enviadas a manufactura.
* `hardware/dev`: Rama base donde se integran los cambios de circuitos, esquemáticos, PCBs y librerías EDA.
* `firmware/dev`: Rama base para integración de código fuente embebido y librerías de control.
* `tools/dev`: Rama base para scripts de telemetría en pista, procesamiento de datos y utilidades de configuración.
* `docs/general`: Rama base para manuales de taller, diagramas de arnés, pinouts y notas técnicas.

---

### 2. Flujo Completo por Terminal

#### Paso 1: Clonar y actualizar el repositorio local
```bash
git clone <URL_DEL_REPOSITORIO>
cd <NOMBRE_DEL_REPO>
git fetch --all
```

#### Paso 2: Crear tu rama personal de trabajo
Sitúate en la rama base de tu disciplina y crea una rama específica para tu tarea:

* **Si vas a diseñar hardware (esquemático/PCB):**
```bash
git checkout hardware/dev
git pull origin hardware/dev
git checkout -b hardware/nombre-tarea
# Ejemplo: git checkout -b hardware/filtro-alimentacion-ecu
```

* **Si vas a programar firmware:**
```bash
git checkout firmware/dev
git pull origin firmware/dev
git checkout -b firmware/nombre-tarea
# Ejemplo: git checkout -b firmware/lectura-pedal-adc
```

* **Si vas a desarrollar scripts / herramientas / telemetría:**
```bash
git checkout tools/dev
git pull origin tools/dev
git checkout -b tools/nombre-tarea
# Ejemplo: git checkout -b tools/parser-telemetria-serial
```

* **Si vas a actualizar documentación o diagramas:**
```bash
git checkout docs/general
git pull origin docs/general
git checkout -b docs/nombre-tarea
# Ejemplo: git checkout -b docs/diagrama-arnes-potencia
```

#### Paso 3: Guardar cambios y crear commits limpios
Agrega únicamente los archivos pertinentes (esquemáticos, código fuente, scripts, modelos 3D). Evita subir archivos temporales, respaldos locales o cachés de compilación:
```bash
git status
git add hardware/ecu/archivo_modificado.kicad_sch
git commit -m "feat(ecu): agregar diodo de proteccion contra polaridad inversa"
```

*Convenciones de mensajes:*
* `feat(módulo):` nueva función, circuito o subsistema.
* `fix(módulo):` corrección de errores, trazos de PCB o bugs de software.
* `docs(módulo):` diagramas de cableado, notas de conexionado o manuales.
* `tools(módulo):` scripts de adquisición, análisis o utilerías de pista.

#### Paso 4: Subir tu rama a GitHub
```bash
git push -u origin <nombre-completo-de-tu-rama>
# Ejemplo: git push -u origin hardware/filtro-alimentacion-ecu
```

---

### 3. Cómo abrir y procesar el Pull Request (PR)

#### Opción A: Desde la interfaz web de GitHub (Método estándar)
1. Entra al enlace del repositorio en GitHub desde tu navegador.
2. Verás un recuadro amarillo con el botón **"Compare & pull request"**. Haz clic ahí. (Si no aparece, entra a la pestaña **Pull requests** y presiona **New pull request**).
3. **Selección de ramas (Punto crítico):**
   * **Base (izquierda):** Selecciona la rama destino según tu área (`hardware/dev`, `firmware/dev`, `tools/dev` o `docs/general`). **No selecciones `main`** a menos que sea una entrega formal de temporada.
   * **Compare (derecha):** Selecciona tu rama de trabajo personal (`hardware/nombre-tarea`, `firmware/nombre-tarea`, etc.).
4. **Título y Descripción:**
   * Título corto y descriptivo: `feat(ecu): Proteccion contra polaridad inversa y desacoplo`.
   * Descripción: Explica qué problema resuelve, qué pruebas realizaste (simulación, osciloscopio o banco de pruebas) y si modificaste algún componente en la BOM.
5. **Revisores:** En la columna lateral derecha, en **Reviewers**, asigna al **Senior Chief Engineer** o al **Capitán de Área Técnica**.
6. Haz clic en **Create pull request**.

#### Opción B: Desde la terminal usando GitHub CLI (`gh`)
Si tienes instalada la herramienta oficial de GitHub en tu consola:
```bash
# Para hardware apuntando a hardware/dev:
gh pr create --base hardware/dev --title "feat(ecu): proteccion inversa" --body "Se agrega diodo y capacitor de desacoplo en entrada de 12V"

# Para firmware apuntando a firmware/dev:
gh pr create --base firmware/dev --title "feat(pedal): calibracion ADC" --body "Lectura lineal de pedal con debounce y promedio movil"

# Para herramientas apuntando a tools/dev:
gh pr create --base tools/dev --title "feat(telemetria): script de lectura serial" --body "Parser de tramas UART y almacenamiento en CSV"

# Para documentación apuntando a docs/general:
gh pr create --base docs/general --title "docs(arnes): pinout conector principal" --body "Actualizacion de diagrama de pines de alimentacion y sensores"
```

#### Paso 5: ¿Qué pasa si te piden correcciones en la revisión?
No cierres el PR ni generes una nueva rama. Aplica las correcciones en tus archivos locales dentro de tu misma rama de trabajo, guarda los cambios y vuelve a enviar el push:
```bash
git add hardware/ecu/archivo_corregido.kicad_sch
git commit -m "fix(ecu): corregir espaciado de pistas segun observaciones"
git push origin <nombre-completo-de-tu-rama>
```
El Pull Request en GitHub se actualizará automáticamente con el nuevo commit. Cuando los revisores den su visto bueno, realizarán el **Merge** formal hacia la rama de desarrollo correspondiente.

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
