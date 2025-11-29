# Robotic Arm RAS – Control con ROS 2, Arduino y Joystick

Sistema completo para controlar un brazo robótico de cuatro servomotores utilizando ROS 2 Jazzy, Arduino y un joystick. El proyecto permite operar el brazo mediante tópicos ROS o mediante un joystick con control continuo de los ejes.


---

## Tabla de contenidos

1. [Instalación rápida](#instalación-rápida)
2. [Introducción](#introducción)
3. [Estructura del proyecto](#estructura-del-proyecto)
4. [Cómo ejecutarlo](#cómo-ejecutarlo)

   * [Modo teclado](#modo-teclado)
   * [Modo joystick](#modo-joystick)
5. [Flujo del código](#flujo-del-código)
6. [Video del Expo-Proyecto](#video-del-expo-proyecto)
7. [Equipo](#equipo)
8. [Licencia](#licencia)

---

## Instalación rápida

1. Clonar el repositorio:

   ```bash
   git clone https://github.com/usuario/robotic_arm_ras.git
   cd robotic_arm_ras
   ```

2. Instalar dependencias ROS 2 (incluyendo soporte para joystick):

   ```bash
   sudo apt install ros-jazzy-desktop ros-jazzy-joy
   ```

3. Cargar el firmware de Arduino:

   * Abrir la carpeta `Arduino_robotic_arm_controller` o `arduino_joystick` en PlatformIO o Arduino IDE.
   * Seleccionar la placa correcta (UNO/Nano/Mega).
   * Compilar y subir el código al Arduino desde el botón de “Upload”.

4. Compilar los paquetes ROS 2 (desde el workspace que contenga los paquetes):

   ```bash
   colcon build
   source install/setup.bash
   ```

5. Ejecutar modo teclado:

   ```bash
   ros2 run robotic_arm_servo_controller joint_controller
   ```

6. Ejecutar modo joystick:

   ```bash
   ros2 run joy joy_node
   ros2 run robotic_arm_joy joint_controller
   ```

---

## Introducción

Este proyecto implementa el control completo de un brazo robótico tipo RAS mediante la integración de ROS 2 y Arduino. A través de nodos en Python y firmware en C++, se controlan cuatro servomotores: base, dos articulaciones y garra.

El sistema incluye dos modos de operación:

* **Modo teclado**: control directo por ángulos usando tópicos ROS.
* **Modo joystick**: control continuo mediante los ejes de un mando ( xbox 360 wired), mapeando los valores de -1 a 1 en cada eje a ángulos de 0° a 180°.

Tecnologías utilizadas:

* ROS 2 Jazzy
* Python 3 (rclpy, sensor_msgs, std_msgs, pyserial)
* Arduino Framework con PlatformIO o Arduino IDE
* C++ (Arduino.h, Servo.h)
* Comunicación serial USB


---

## Estructura del proyecto

```text
Arduino_robotic_arm_controller/
 ├── src/
 │   └── main.cpp

robotic_arm_servo_controller/
 ├── package.xml
 ├── resource/
 ├── robotic_arm_servo_controller/
 │   └── joints_keyboard.py
 ├── setup.py

arduino_joystick/
 ├── src/
 │   └── main.cpp

robotic_arm_joy/
 ├── package.xml
 ├── resource/
 ├── robotic_arm_joy/
 │   └── robotic_arm_joy.py
 ├── setup.py
```


### Arduino_robotic_arm_controller (modo teclado)

* Archivo principal: `Arduino_robotic_arm_controller/src/main.cpp`.
* Recibe cadenas por serial con dos tipos de formato:

  * `"X,Y,Z"` → ángulos de los tres servos principales.
  * `"open"` / `"close"` → control del servo de la garra.
* Cada ángulo se limita al rango [0°, 180°] antes de escribir en los servos.

### robotic_arm_servo_controller (modo teclado)

* Nodo ROS 2 en Python definido en `robotic_arm_servo_controller/joints_keyboard.py`.
* Nombre del nodo: `joint_controller`.
* Suscripciones:

  * `/joint_cmd` (`std_msgs/String`) con formato `"X,Y,Z"`.
  * `/gripper_cmd` (`std_msgs/String`) con valores `"open"` o `"close"`.
* Valida el formato de los mensajes, agrega el terminador `\n` y los envía al puerto serial del Arduino a 9600 baudios (según la configuración del firmware del modo teclado).

### arduino_joystick (modo joystick)

* Archivo principal: `arduino_joystick/src/main.cpp`.

* Configura cuatro servos:

  * `servo1` (base) en el pin 7.
  * `servo2` (servo derecho) en el pin 6.
  * `servo3` (servo izquierdo vertical) en el pin 5.
  * `servo4` (garra) en el pin 4.

* Recibe líneas con formato:

  ```text
  eje_der,eje_izq_ver,eje_der_ver,gripper_state
  ```

* Cada eje se interpreta como valor en el rango [-1, 1] y se transforma a ángulo [0°, 180°] mediante:

  ```cpp
  ang = (x + 1.0f) * 90.0f;
  ```

* `gripper_state` controla la posición del servo de la garra:

  * `0` → garra cerrada (0°).
  * `1` → garra abierta (45°).

* Velocidad de comunicación: 115200 baudios.

### robotic_arm_joy (modo joystick)

* Paquete ROS 2 definido en `robotic_arm_joy`.

* Nodo principal: `robotic_arm_joy` (clase `Node`).

* Consola de ejecución:

  ```bash
  ros2 run robotic_arm_joy joint_controller
  ```

* Suscripción: `/joy` (`sensor_msgs/Joy`).

* Lógica principal:

  * Lee los ejes del joystick:

    * `axes[1]` → eje vertical del stick izquierdo.
    * `axes[3]` → eje horizontal del stick derecho.
    * `axes[4]` → eje vertical del stick derecho.

  * Lee el botón A (`buttons[0]`) y lo utiliza como toggle de la garra: cada pulsación alterna entre abierto y cerrado.

  * Construye la cadena:

    ```text
    eje_der,eje_izq_ver,eje_der_ver,gripper_state\n
    ```

  * Envía la cadena al Arduino a través de `/dev/ttyACM0` a 115200 baudios.



---

## Cómo ejecutarlo

### Modo teclado

1. Conectar el Arduino con el firmware de `Arduino_robotic_arm_controller` cargado.

2. Iniciar el nodo controlador:

   ```bash
   ros2 run robotic_arm_servo_controller joint_controller
   ```

3. Enviar comandos de prueba:

   * Mover los tres servos principales:

     ```bash
     ros2 topic pub /joint_cmd std_msgs/String "data: '90,45,120'"
     ```

   * Abrir la garra:

     ```bash
     ros2 topic pub /gripper_cmd std_msgs/String "data: 'open'"
     ```

   * Cerrar la garra:

     ```bash
     ros2 topic pub /gripper_cmd std_msgs/String "data: 'close'"
     ```

### Modo joystick

1. Cargar en el Arduino el firmware de `arduino_joystick`.

2. Conectar el joystick al PC.

3. Ejecutar el nodo del joystick de ROS 2:

   ```bash
   ros2 run joy joy_node
   ```

4. Ejecutar el nodo de control del brazo:

   ```bash
   ros2 run robotic_arm_joy joint_controller
   ```

5. Control del brazo:

   * Stick izquierdo vertical (`axes[1]`) → servo izquierdo vertical.
   * Stick derecho horizontal (`axes[3]`) → servo de la base.
   * Stick derecho vertical (`axes[4]`) → servo derecho.
   * Botón A (`buttons[0]`) → alterna entre abrir y cerrar la garra.



---

## Flujo del código

### Resumen general

1. El usuario interactúa mediante teclado o joystick.
2. Un nodo ROS 2 genera comandos con un formato específico.
3. El comando se envía al Arduino mediante comunicación serial.
4. El firmware de Arduino parsea la cadena, calcula los ángulos y actualiza los servomotores.


### Flujo en el modo teclado

1. Se publica un mensaje en `/joint_cmd` o `/gripper_cmd`.
2. El nodo `joint_controller` de `robotic_arm_servo_controller` valida el contenido del string.
3. El nodo añade `\n` y escribe el comando en el puerto serial configurado para el Arduino.
4. El firmware de `Arduino_robotic_arm_controller`:

   * Para `"X,Y,Z"`: separa los valores, los convierte a enteros, los limita y los escribe en los tres servos principales.
   * Para `"open"` / `"close"`: ajusta el ángulo del servo de la garra.

### Flujo en el modo joystick

1. El nodo `joy_node` publica mensajes `Joy` en `/joy` con ejes y botones.

2. El nodo `joint_controller` del paquete `robotic_arm_joy`:

   * Lee los ejes relevantes (`axes[1]`, `axes[3]`, `axes[4]`).
   * Detecta pulsaciones del botón A para cambiar el estado de `gripper_open`.
   * Asigna `gripper_state = 0` o `1` según el estado de la garra.
   * Construye la línea `eje_der,eje_izq_ver,eje_der_ver,gripper_state\n`.
   * Envía la línea por el puerto serial `/dev/ttyACM0`.

3. El firmware de `arduino_joystick`:

   * Separa la cadena en cuatro campos.
   * Convierte los tres primeros a `float` y los mapea a ángulos [0°, 180°] con `mapAxisToAngle`.
   * Actualiza los servos de base, servo derecho y servo izquierdo vertical.
   * Ajusta el servo de la garra a 0° (cerrada) o 45° (abierta) según `gripper_state`.



---

## Video del Expo-Proyecto
```markdown
[![Ver video de demostración del brazo robótico](https://img.youtube.com/vi/hoDyDBRrRAo/0.jpg)](https://youtu.be/hoDyDBRrRAo)

```

También se pueden incluir miniaturas o capturas de pantalla en la carpeta `media/` para documentar diferentes posiciones del brazo y pruebas realizadas.

---

## Equipo

* Marlon Jhoan Garcia Restrepo – Estudiante de Ingeniería de Sistemas – [mjgarcia@javeriana.edu.co](mailto:mjgarcia@javeriana.edu.co)
* Isabella Hermosa Losada – Estudiante de Ingeniería de Sistemas – [ishermosa@javeriana.edu.co](mailto:ishermosa@javeriana.edu.co)
* Luis Felipe Rincón Sierra – Estudiante de Ingeniería Mecatrónica – [lfelipe-rincon@javeriana.edu.co](mailto:lfelipe-rincon@javeriana.edu.co)

---

## Licencia

Este proyecto se distribuye bajo los términos de la licencia especificada en el archivo `LICENSE` ubicado en la raíz del repositorio.
