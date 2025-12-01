import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

# joystick
eje_izq = 0.0
eje_izq_ver = 0.0
eje_der = 0.0
eje_der_ver = 0.0

# toggle para gancho
boton_a_prev = 0
gripper_open = False  # False = cerrado, True = abierto

# nodo global
node = None

# serial global
serial_port = None


def enviar_a_arduino():
    global serial_port, eje_izq, eje_izq_ver, eje_der, eje_der_ver, gripper_state

    if serial_port is None:
        return

    # aqui mandas los ejes y el estado de la garra
    linea = f"{eje_der},{eje_izq_ver},{eje_der_ver},{gripper_state}\n"
    try:
        serial_port.write(linea.encode("utf-8"))
    except Exception as e:
        node.get_logger().error(f"Error enviando al Arduino: {e}")


def joy_callback(msg: Joy):
    global eje_izq, eje_izq_ver, eje_der, eje_der_ver
    global boton_a_prev, gripper_open
    global gripper_state

    # joysticks
    eje_izq_ver = msg.axes[1] if len(msg.axes) > 1 else 0.0
    eje_der = msg.axes[3] if len(msg.axes) > 3 else 0.0     
    eje_der_ver = msg.axes[4] if len(msg.axes) > 4 else 0.0

    # boton A
    boton_a = msg.buttons[0] if len(msg.buttons) > 0 else 0

    # toggle de "A" cuando pasa de 0 a 1
    if boton_a_prev == 0 and boton_a == 1:
        gripper_open = not gripper_open

    # actualizar estado previo
    boton_a_prev = boton_a

    # actualizar gripper_state segun gripper_open 
    if gripper_open:
        gripper_state = 1
    else:
        gripper_state = 0

    # log para revisar
    node.get_logger().info(
        f"eje_izq_ver={eje_izq_ver:.2f} | eje_der={eje_der:.2f} | "
        f"eje_der_ver={eje_der_ver:.2f} | gripper_state={gripper_state}"
    )

    enviar_a_arduino()


def main(args=None):
    global node, serial_port

    rclpy.init(args=args)

    node = Node("robotic_arm_joy")
    node.get_logger().info("Leyendo joystick")

    try:
        serial_port = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        time.sleep(2)
        node.get_logger().info("Conectado al Arduino")
    except Exception as e:
        node.get_logger().error(f"No se pudo abrir serial: {e}")
        serial_port = None

    node.create_subscription(Joy, "/joy", joy_callback, 10)

    rclpy.spin(node)

    if serial_port:
        serial_port.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()