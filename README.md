# Robotic Arm RAS – Control con ROS 2, Arduino y Joystick

Sistema completo para controlar un brazo robótico de cuatro servomotores utilizando ROS 2 Jazzy, Arduino y un joystick. El proyecto permite operar el brazo mediante publicaciones de tópicos ROS o mediante un joystick para control incremental.

---

## Autores

* Marlon Jhoan Garcia Restrepo – Ingeniería de Sistemas – [mjgarcia@javeriana.edu.co](mailto:mjgarcia@javeriana.edu.co)
* Isabella Hermosa Losada – Ingeniería de Sistemas – [ishermosa@javeriana.edu.co](mailto:ishermosa@javeriana.edu.co)
* Luis Felipe Rincón Sierra – Ingeniería Mecatrónica – [lfelipe-rincon@javeriana.edu.co](mailto:lfelipe-rincon@javeriana.edu.co)

---

# Instalación rápida

### 1. Clonar el repositorio

```bash
git clone https://github.com/usuario/robotic_arm_ras.git
cd robotic_arm_ras
```

### 2. Instalar dependencias ROS 2

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-fastrtps-cpp
sudo apt install ros-jazzy-joy
```

### 3. Instalar dependencias Python

```bash
pip install pyserial
```

### 4. Instalar PlatformIO

Desde Visual Studio Code, instalar la extensión “PlatformIO IDE”.

### 5. Cargar el firmware de Arduino (modo teclado o modo joystick)

Modo teclado:

```bash
cd Arduino_robotic_arm_controller
pio run --target upload
```

Modo joystick:

```bash
cd arduino_joystick
pio run --target upload
```

### 6. Compilar los paquetes ROS 2

```bash
colcon build
source install/setup.bash
```

### 7. Ejecutar control por teclado

```bash
ros2 run robotic_arm_servo_controller joint_controller
```

### 8. Ejecutar control por joystick

```bash
ros2 run joy joy_node
ros2 run robotic_arm_joy joint_controller
```

---

# 1. Introducción

El proyecto implementa el control completo del brazo robótico RAS mediante ROS 2 y Arduino. Incluye dos modos de operación: control por ángulos enviados a través de tópicos y control incremental desde un joystick. La comunicación se realiza mediante puerto serial y firmware en C++ que interpreta los comandos enviados desde ROS.

Tecnologías utilizadas:

* ROS 2 Jazzy
* Python 3 (rclpy, sensor_msgs, std_msgs, pyserial)
* Arduino Framework con PlatformIO
* C++ (Arduino.h, Servo.h)
* Comunicación serial USB

---

# 2. Estructura del proyecto

```
Arduino_robotic_arm_controller/
 ├── src/main.cpp

robotic_arm_servo_controller/
 ├── robotic_arm_servo_controller/joints_keyboard.py
 ├── setup.py

arduino_joystick/
 ├── src/main.cpp

robotic_arm_joy/
 ├── robotic_arm_joy/robotic_arm_joy.py
 ├── setup.py
```

## Descripción de módulos

### Arduino_robotic_arm_controller (modo teclado)

Archivo principal: `Arduino_robotic_arm_controller/src/main.cpp`
Recibe cadenas con formato:

```
X,Y,Z
open
close
```

Procesa los valores, mueve tres servos principales y controla el servo de la garra.

### robotic_arm_servo_controller (modo teclado)

Nodo ROS: `joint_controller`
Archivo: `robotic_arm_servo_controller/joints_keyboard.py`

Suscripciones:

* `/joint_cmd` → formato `"X,Y,Z"`
* `/gripper_cmd` → `"open"` o `"close"`

Envía cada comando por serial al Arduino.

### arduino_joystick (modo joystick)

Archivo principal: `arduino_joystick/src/main.cpp`
Recibe cadenas con formato:

```
eje_der,eje_izq_ver,eje_der_ver,gripper_state
```

Convierte los ejes del joystick en incrementos sobre los ángulos actuales de los servos.

### robotic_arm_joy (modo joystick)

Nodo ROS: `joint_controller`
Archivo: `robotic_arm_joy/robotic_arm_joy.py`

Suscripción:

* `/joy` → mensaje `sensor_msgs/Joy`

Genera la cadena incremental con el estado de la garra y los ejes, y la envía al Arduino.

---

# 3. Cómo ejecutarlo

---

## 3.1 Requerimientos de hardware

* Arduino UNO/Nano/Mega
* Cuatro servomotores (base, articulación izquierda, articulación derecha, garra)
* Fuente externa adecuada
* PC con ROS 2 Jazzy
* Joystick compatible con ros2_joy (modo joystick)

---

## 3.2 Requerimientos de software

* ROS 2 Jazzy
* Python 3.10+
* pyserial
* PlatformIO IDE
* Paquete `joy` para ROS 2

---

## 3.3 Modo teclado (control por tópicos ROS)

### 1. Cargar firmware

```bash
cd Arduino_robotic_arm_controller
pio run --target upload
```

### 2. Ejecutar el nodo controlador

```bash
ros2 run robotic_arm_servo_controller joint_controller
```

### 3. Enviar comandos

Mover servos:

```bash
ros2 topic pub /joint_cmd std_msgs/String "data: '90,45,120'"
```

Abrir garra:

```bash
ros2 topic pub /gripper_cmd std_msgs/String "data: 'open'"
```

Cerrar garra:

```bash
ros2 topic pub /gripper_cmd std_msgs/String "data: 'close'"
```

---

## 3.4 Modo joystick (control incremental)

### 1. Cargar firmware

```bash
cd arduino_joystick
pio run --target upload
```

### 2. Ejecutar nodo de joystick

```bash
ros2 run joy joy_node
```

### 3. Ejecutar nodo controlador del brazo

```bash
ros2 run robotic_arm_joy joint_controller
```

### 4. Controles

* Stick izquierdo vertical → servo 2
* Stick derecho horizontal → servo 1
* Stick derecho vertical → servo 3
* Botón A → alterna abrir/cerrar garra

---

# 4. Flujo del código

## Modo teclado

1. `/joint_cmd` recibe `"X,Y,Z"`.
2. El nodo `joint_controller` valida y envía `"X,Y,Z\n"` por serial.
3. El Arduino parsea los ángulos y mueve los servos correspondientes.
4. `/gripper_cmd` recibe `"open"` o `"close"` y el Arduino actualiza el servo de la garra.

## Modo joystick

1. `/joy` publica ejes del control.
2. El nodo `joint_controller` del paquete `robotic_arm_joy` interpreta ejes y botón A.
3. Se envía la cadena `"eje_der,eje_izq_ver,eje_der_ver,gripper_state\n"`.
4. El Arduino incrementa los ángulos y mueve los servos.

## Librerías utilizadas

### ROS 2 (Python)

* rclpy
* sensor_msgs.msg.Joy
* std_msgs.msg.String
* serial
* time

### Arduino (C++)

* Arduino.h
* Servo.h
* String

---

# 5. Video del Expo-Proyecto

```
[Agregar enlace o archivo de demostración]
```

---

# 6. Equipo

* Marlon Jhoan Garcia Restrepo
* Isabella Hermosa Losada
* Luis Felipe Rincón Sierra

