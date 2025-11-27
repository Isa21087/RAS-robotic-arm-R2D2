#include <Arduino.h>
#include <Servo.h>

Servo servo1;  // servo de abajo (base)
Servo servo2;  // servo derecho
Servo servo3;  // servo izquierdo vertical
Servo servo4; //Servo gripper

// pines de los servos
const int pin_servo1 = 7;
const int pin_servo2 = 6;
const int pin_servo3 = 5;

// pin para estado de garra (opcional, por ejemplo un led o rele)
const int pin_garra = 4;

// funcion para pasar de eje [-1,1] a angulo [0,180]
int mapAxisToAngle(float x) {
    int ang = (int)((x + 1.0f) * 90.0f);  // -1 -> 0, 0 -> 90, 1 -> 180

    if (ang < 0) {
        ang = 0;
    }
    if (ang > 180) {
        ang = 180;
    }
    return ang;
}

void setup() {

    Serial.begin(115200);  // igual que en ROS

    servo1.attach(pin_servo1);
    servo2.attach(pin_servo2);
    servo3.attach(pin_servo3);
    servo4.attach(pin_garra);

    // posiciones iniciales
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(45);
}

void loop() {

    if (Serial.available() > 0) {

        // formato esperado:
        // eje_der,eje_izq_ver,eje_der_ver,gripper_state\n
        String linea = Serial.readStringUntil('\n');
        linea.trim();

        if (linea.length() == 0) {
            return;
        }

        // posiciones de las comas
        int c1 = linea.indexOf(',');
        int c2 = linea.indexOf(',', c1 + 1);
        int c3 = linea.indexOf(',', c2 + 1);

        // deben existir 3 comas
        if (c1 == -1 || c2 == -1 || c3 == -1) {
            return;
        }

        // separar campos
        String s_eje_der      = linea.substring(0, c1);
        String s_eje_izq_ver  = linea.substring(c1 + 1, c2);
        String s_eje_der_ver  = linea.substring(c2 + 1, c3);
        String s_gripper      = linea.substring(c3 + 1);

        // convertir a numeros
        float eje_der     = s_eje_der.toFloat();
        float eje_izq_ver = s_eje_izq_ver.toFloat();
        float eje_der_ver = s_eje_der_ver.toFloat();
        int gripper_state = s_gripper.toInt();

        // mapear ejes a angulos
        int ang1 = mapAxisToAngle(eje_der);      // servo 1 abajo
        int ang2 = mapAxisToAngle(eje_izq_ver);  // servo 2 izquierda vertical
        int ang3 = mapAxisToAngle(eje_der_ver);  // servo 3 derecha

        // mover servos
        servo1.write(ang1);
        servo2.write(ang2);
        servo3.write(ang3);

        // usar gripper_state como salida digital
        // 0 -> LOW, 1 -> HIGH
        if (gripper_state == 1) {
            servo4.write(45);
        } else {
            servo4.write(0);
        }
    }
}