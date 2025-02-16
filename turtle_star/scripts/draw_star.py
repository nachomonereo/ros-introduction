#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
import math

def move_turtle(pub, linear, angular, duration):
    """ Función para mover la tortuga con velocidad lineal y angular por un tiempo. """
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(twist)
        rospy.sleep(0.1)  # Pequeño delay para evitar saturar el tópico

    # Detener la tortuga al final de cada movimiento
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

def draw_star():
    rospy.init_node('turtle_star', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # Espera a que el nodo y el publisher se inicialicen

    # Teletransportar la tortuga al centro
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        teleport(5.5, 5.5, 0)  # Centro de la pantalla
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al teletransportar: {e}")

    rospy.sleep(1)  # Pausa para asegurar la teletransportación

    # Parámetros de la estrella
    side_length = 2.0  # Tamaño de cada segmento de la estrella
    angle = 144  # Ángulo entre las puntas de la estrella

    for _ in range(5):
        move_turtle(pub, side_length, 0, 1.0)  # Mover hacia adelante
        move_turtle(pub, 0, math.radians(angle), 1.0)  # Girar

    rospy.loginfo("Dibujo de estrella completado.")

if __name__ == '__main__':
    try:
        draw_star()
    except rospy.ROSInterruptException:
        pass

