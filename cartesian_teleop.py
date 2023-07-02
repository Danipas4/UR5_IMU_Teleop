#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from std_msgs.msg import String

import rospy

from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math
import roboticstoolbox as rtb
from spatialmath import SE3

import numpy as np

# Topic de donde se recibe información
imu_topic = '/imu'
# Inicialización de variables
alpha = 0
beta = 0
joystick = 0
mode = 0           
prev_but = 1
# Ganancias
step_size_1 = 0.0005
step_size_2 = 0.0005
step_size_3 = 0.00005
# Offsets de los giros 
offset_x = 0.47
offset_y = 2.51
offset_joystick = 500      # En la posición central, va de [0,1000]

# Crear el modelo UR5
robot = rtb.models.UR5()
#Posición articular inicial 
q0 = [0.0, -1.5, 1.0, 0.0, 0.0, 0.0]
# #Visualizar parámetros
# print(robot)

def publish_joint_positions():
    
    # Se configura la parte de Publisher
    rospy.init_node('joint_positions_publisher')
    pub = rospy.Publisher('/eff_joint_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    rate = rospy.Rate(10)  # Frecuencia de publicación (10 Hz en este caso)
    
    # # ---------------------------CÓDIGO PARA ENCONTRAR POSICIÓN INICIO------------------------------------
    # # Para obtener una posición de inicio
    # # Se coloca el robot moviendo las articulaciones en una posición razonable
    # joint_positions = [0.0, -1.5, 1.0, 0.0, 0.0, 0.0] #Home
    # #Cinematica directa para obtener la matriz de transformación homogénea
    # pose = robot.fkine(joint_positions)
    # print(pose)
    # # 0         0.8776    0.4794    0.4197    
    # # 1         0         0         0.1915    
    # # 0         0.4794   -0.8776    0.6181    
    # # 0         0         0         1        
    # # Matriz de rotación
    # R = np.array([
    #     [0, 0.8776, 0.4794, 0.4197],
    #     [1, 0, 0, 0.1915],
    #     [0, 0.4794, -0.8776, 0.6181],
    #     [0, 0, 0, 1]
    # ])

    # # Obtener Roll Pitch Yaw
    # roll = np.arctan2(R[2, 1], R[2, 2])
    # pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    # yaw = np.arctan2(R[1, 0], R[0, 0])

    # print("Roll: ", roll)
    # print("Pitch: ", pitch)
    # print("Yaw: ", yaw)

    # # Roll:  2.6416234260044384
    # # Pitch:  -0.0
    # # Yaw:  1.5707963267948966
    # # -----------------------------------------------------------------------------------------------------------
    
    # Posición y orientación iniciales
    X = 0.4197 # Posición x
    Y = 0.1915  # Posición Y
    Z = 0.6181 # Posición Z
    roll = 2.6416234260044384  # Ángulo de rotación alrededor del eje x
    pitch = -0.0  # Ángulo de rotación alrededor del eje y
    yaw = 1.5707963267948966  # Ángulo de rotación alrededor del eje z

    pose = [X , Y, Z, roll, pitch, yaw]
    
    # Se configura la parte de Subscriber 
    rospy.Subscriber(imu_topic, String, imu_callback, pose)

    while not rospy.is_shutdown():
        # Obtener las posiciones articulares del robot (reemplazar con la lógica correspondiente)
        joint_positions = get_joint_positions(pose)
        global q0
        q0 = joint_positions
        # Crear el mensaje de objetivo de acción de trayectoria
        goal = FollowJointTrajectoryActionGoal()

        # Crear el mensaje de trayectoria
        trajectory = JointTrajectory()
        trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Crear un punto de trayectoria y asignar las posiciones articulares
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(1.0)  # Duración de la trayectoria
        
        # Agregar el punto de trayectoria a la trayectoria
        trajectory.points = [point]

        # Asignar la trayectoria al mensaje de objetivo de acción
        goal.goal.trajectory = trajectory

        # Publicar el mensaje de objetivo de acción en el topic
        pub.publish(goal)
        rate.sleep()

def get_joint_positions(pose):
    

    # Definir pose cartesiana objetivo
    Tep = SE3.Trans(pose[0], pose[1], pose[2]) * SE3.RPY(pose[3], pose[4], pose[5])

    # Cinematica inversa
    global q0
    sol = robot.ik_LM(Tep, q0=q0)         
    # print(sol)

    # Se guarda la posición articular objetivo
    q_objetivo = sol[0] 
    # print(q_pickup[0])
    # print(robot.fkine(q_pickup))    # FK shows that desired end-effector pose was achieved
    # robot.fkine(q_objetivo)

    return q_objetivo

def imu_callback(imu, pose):
    #rospy.loginfo(imu.data)

    Ax = float(imu.data[imu.data.index('A')+1:imu.data.index('B')])
    Ay = float(imu.data[imu.data.index('B')+1:imu.data.index('C')])
    Joy_value = float(imu.data[imu.data.index('C')+1:imu.data.index('D')])
    But = float(imu.data[imu.data.index('D')+1:imu.data.index('E')])

    global alpha, beta, mode, joystick, prev_but

    # Si se ha pulsado el botón se cambia de modo
    if prev_but != But and But == 0: 
        
        mode = not mode
        prev_but = But

        rospy.loginfo("Mode changed")

    elif But == 1: 
        prev_but = But
    
	# Se calculan los valores restando un offset 
    alpha = Ax - offset_x
    beta = Ay - offset_y
    joystick = Joy_value - offset_joystick

	# Gestión de datos
    # Converting into integer
    alpha = int(-alpha)
    beta = int(-beta)
    joystick = int(joystick)

    control_1 = round(step_size_1 * alpha,2)
    control_2 = round(step_size_2 * beta,2)
    control_3 = round(step_size_3 * joystick,2)

    # print("Control_1: ",control_1)
    # print("Control_2: ",control_2)
    # print("Control_3: ",control_3)
    
    # Asignación de velocidades para publicar: 
    if mode == 0: 
        pose[0] += control_1

        pose[1] += control_2

        pose[2] += control_3
    
    elif mode == 1: 
        pose[3] += control_1

        pose[4] += control_2

        pose[5] += control_3

    else: 
        rospy.loginfo("Incorrect MODE")


if __name__ == '__main__':
    try:
        publish_joint_positions()
    except rospy.ROSInterruptException:
        pass