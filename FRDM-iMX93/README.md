# Guía de Configuración: i.MX93 EVK con VS Code, Red Punto a Punto y Optimización de Memoria

Esta guía documenta la configuración completa de la placa **NXP i.MX93 EVK** para desarrollo con conexión directa por cable Ethernet hacia la PC, asignación automática de IP por DHCP (vía `dnsmasq`), autenticación por llave SSH, expansión de almacenamiento, memoria Swap y conexión remota con Visual Studio Code.

---

## 1. Conexión de Red Punto a Punto (IP Estática y DHCP en i.MX93)

Para que la i.MX93 funcione de forma autónoma al conectarla por cable de red a la PC (sin router externo), la placa se configura con una IP estática fija y un servidor DHCP (`dnsmasq`) que le entrega automáticamente una IP compatible a la PC host.

### Archivo de Configuración de Red y DHCP (`imxrouter.conf`)
En el directorio raíz `/root/` de la placa se creó el archivo `imxrouter.conf`:

```ini
# Configuración para dnsmasq en modo router punto a punto
interface=eth0
bind-interfaces
dhcp-range=192.168.10.10,192.168.10.50,12h
dhcp-option=3,192.168.10.1
dhcp-option=6,8.8.8.8
```

### Script de Inicialización de Red
Para levantar la interfaz con la IP fija `192.168.10.1` y arrancar el servidor DHCP:

```bash
# 1. Asignar IP fija y máscara a la interfaz Ethernet de la placa
ip addr add 192.168.10.1/24 dev eth0
ip link set eth0 up

# 2. Iniciar dnsmasq con el archivo de configuración
dnsmasq -C /root/imxrouter.conf
```

> **Resultado en la PC:** Windows detecta la conexión por Ethernet y recibe automáticamente una IP en el rango `192.168.10.x` (como `192.168.10.10`), permitiendo hacer `ping 192.168.10.1` de inmediato sin modificar el adaptador de Windows.

---

## 2. Autenticación SSH sin Contraseña (Llaves RSA)

Para habilitar login directo y seguro desde Visual Studio Code sin pedir contraseña en cada reconexión:

### En la PC Host (PowerShell o CMD)
1. Generar el par de llaves (si no existe):
   ```cmd
   ssh-keygen -t rsa -b 4096
   ```
2. Transferir la llave pública a la placa:
   ```cmd
   type %USERPROFILE%\.ssh\id_rsa.pub | ssh root@192.168.10.1 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
   ```
3. Comprobar acceso directo:
   ```cmd
   ssh root@192.168.10.1
   ```

---

## 3. Configuración de Visual Studio Code (Remote - SSH)

### Archivo de Configuración SSH (`~/.ssh/config`)
En la PC, edita `C:\Users\<TuUsuario>\.ssh\config` para registrar el host y configurar los parámetros de latido (*KeepAlive*), evitando que la conexión caiga por inactividad:

```ssh-config
Host imx93
    HostName 192.168.10.1
    User root
    IdentityFile ~/.ssh/id_rsa
    ServerAliveInterval 10
    ServerAliveCountMax 3
```

### Conexión
1. En VS Code, abre la paleta de comandos (`Ctrl + Shift + P`).
2. Selecciona **Remote-SSH: Connect to Host...** y haz clic en **`imx93`**.
3. Abre la carpeta `/root/` desde `File > Open Folder...`.

---

## 4. Expansión de Partición de la MicroSD

Las imágenes BSP grabadas en la MicroSD vienen por defecto con tamaño fijo (~8.6 GB), dejando el resto de la tarjeta sin usar:

```bash
# Redimensionar la partición raíz en /dev/mmcblk1p2 al 100% del disco
parted -s /dev/mmcblk1 resizepart 2 100%
resize2fs /dev/mmcblk1p2

# Verificar que la raíz ahora cuente con ~29 GB (~20 GB libres)
df -h /
```

---

## 5. Creación y Activación de Memoria Swap (2 GB)

Para evitar que el kernel active el *OOM Killer* y cuelgue la placa debido al consumo de RAM del servidor de VS Code y extensiones de Python:

```bash
# 1. Crear el archivo de intercambio de 2 GB
dd if=/dev/zero of=/swapfile bs=1M count=2048

# 2. Permisos y formato
chmod 600 /swapfile
mkswap /swapfile
swapon /swapfile

# 3. Hacerlo persistente en cada arranque
echo '/swapfile none swap sw 0 0' >> /etc/fstab

# 4. Validar estado (debe mostrar ~2047 MB en Swap)
free -m
```

---

## 6. Optimización de VS Code para Placas Embebidas

Para evitar sobrecargar los núcleos Cortex-A55 y la memoria RAM:

1. Ve a **Settings** (`Ctrl + ,`) y selecciona la pestaña **Remote [SSH: imx93]**.
2. Ajusta:
   - `python.languageServer`: `None` (o `Jedi`).
   - `search.followSymlinks`: `false`.
   - `files.watcherExclude`: agregar `**/.vscode-server/**`.

---

## 7. Desconexión y Apagado Seguro

Para evitar corromper los bloques del sistema de archivos en la tarjeta MicroSD:

1. En VS Code, haz clic en la esquina inferior izquierda (`SSH: imx93`) y presiona **Close Remote Connection**.
2. En la terminal de la placa, apaga el sistema de forma controlada:
   ```bash
   poweroff
   ```
3. Espera 10 segundos a que los LEDs de actividad se detengan por completo antes de retirar el cable de alimentación.