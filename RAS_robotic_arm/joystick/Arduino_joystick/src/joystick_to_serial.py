#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import threading
import time

class JoystickToSerialNode(Node):
    def __init__(self):
        super().__init__('joystick_to_serial')
        
        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('threshold', 0.1)
        self.declare_parameter('deadband', 0.05)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.threshold = self.get_parameter('threshold').value
        self.deadband = self.get_parameter('deadband').value
        
        # Variables de control
        self.last_command = '0'
        self.connected = False
        self.ser = None
        
        # Intentar conexión serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Esperar a que Arduino se reinicie
            self.connected = True
            self.get_logger().info(f'Conectado a {self.port} @ {self.baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Error al conectar al puerto serial: {e}')
            self.connected = False
        
        # Suscriptor al joystick
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Timer para lectura de respuestas del Arduino
        self.create_timer(0.1, self.read_arduino_feedback)
        
        self.get_logger().info('Nodo Joystick→Serial listo 🎮')
    
    def joy_callback(self, msg: Joy):
        """Procesa datos del joystick Xbox"""
        if not self.connected:
            return
        
        # Usar stick izquierdo (axes[0] = eje X, axes[1] = eje Y)
        if len(msg.axes) > 0:
            eje_x = msg.axes[0]
            
            # Aplicar deadband para evitar ruido
            if abs(eje_x) < self.deadband:
                eje_x = 0.0
            
            # Determinar comando
            if eje_x > self.threshold:
                new_command = '1'  # Derecha (+)
            elif eje_x < -self.threshold:
                new_command = '-'  # Izquierda (-)
            else:
                new_command = '0'  # Parado
            
            # Solo enviar si cambió el comando
            if new_command != self.last_command:
                self.send_command(new_command)
                self.last_command = new_command
    
    def send_command(self, cmd: str):
        """Envía comando al Arduino"""
        if self.connected and self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
                self.get_logger().debug(f'Enviado: {cmd}')
            except Exception as e:
                self.get_logger().error(f'Error escribiendo serial: {e}')
                self.connected = False
    
    def read_arduino_feedback(self):
        """Lee respuestas del Arduino"""
        if self.connected and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    response = self.ser.readline().decode().strip()
                    if response:
                        self.get_logger().info(f'Arduino: {response}')
            except Exception as e:
                self.get_logger().debug(f'Error leyendo serial: {e}')
    
    def destroy_node(self):
        """Limpieza al cerrar"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickToSerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
