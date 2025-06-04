# The entire code was done by me, whereas the execution of all the 3 tasks of lab 3 were done by me and my lab partner collectively.
import numpy as np
from robolink import *
from robodk import *

RDK = Robolink()

robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception('No robot selected or available')
RUN_ON_ROBOT = False

if RDK.RunMode() != RUNMODE_SIMULATE:
    RUN_ON_ROBOT = False
if RUN_ON_ROBOT:

    success = robot.Connect()
    status, status_msg = robot.ConnectedState()
    if status != ROBOTCOM_READY:
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)

    RDK.setRunMode(RUNMODE_RUN_ROBOT)

joints_ref = robot.Joints()

target_ref = robot.Pose()
pos_ref = target_ref.Pos()

robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(robot.PoseTool())

def transl_rot(Screw_wrt_TrayFrame, Screw_wrt_RobotFrame):
    
    (X_T1, Y_T1), (X_T2, Y_T2) = Screw_wrt_TrayFrame
    (X_R1, Y_R1), (X_R2, Y_R2) = Screw_wrt_RobotFrame

    # Defining the difference in the Angle of rotation 
    theta_T = np.arctan2(Y_T2 - Y_T1, X_T2 - X_T1)
    theta_R = np.arctan2(Y_R2 - Y_R1, X_R2 - X_R1)
    Del_theta = theta_R - theta_T

    # Rotation Matrix
    Rot_mat = np.array([
        [np.cos(Del_theta), -np.sin(Del_theta)],
        [np.sin(Del_theta), np.cos(Del_theta)]
    ])

    # Robot_Point_1 = Rot_mat * Tray_Point_1 + Transformation_mat
    Tray_Point_1 = np.array([X_T1, Y_T1])
    Robot_Point_1 = np.array([X_R1, Y_R1])
    Transformation_mat = Robot_Point_1 - Rot_mat @ Tray_Point_1

    return Rot_mat, Transformation_mat, np.degrees(Del_theta)


def Points_Transformation(Tray_Pose, Rotation, Translation, Angle_in_Deg):
   
    UR5_Pose = []
    for (x_T, y_T) in Tray_Pose:
        Tray_Pts = np.array([x_T, y_T])
        Robot_Pts = Rotation @ Tray_Pts + Translation

        # For this example: Z = 25 mm, Rx = Ry = 0, Rz = rotation angle (in degrees)
        UR5_Pose.append((Robot_Pts[0], Robot_Pts[1], 25, 180, 0, Angle_in_Deg))
    return UR5_Pose 

# Defining the position of each ball placed on the tray with respect to the tray coordinate frame (in mm)
Balls_wrt_Tray = [
    (1 * 25.4, 1 * 25.4), (3 * 25.4, 1 * 25.4),
    (5 * 25.4, 1 * 25.4), (7 * 25.4, 1 * 25.4),

    (1 * 25.4, 3 * 25.4), (3 * 25.4, 3 * 25.4),
    (5 * 25.4, 3 * 25.4), (7 * 25.4, 3 * 25.4),

    (1 * 25.4, 5 * 25.4), (3 * 25.4, 5 * 25.4),
    (5 * 25.4, 5 * 25.4), (7 * 25.4, 5 * 25.4)
]

# For Tray 1: Screw locations of tray 1 in Tray coordinate frame and Robot coordinate frame (in mm)
Screws_wrt_Tray_1 = [(0, 0), (204.3, 0)] 
Screws_wrt_Robot = [(190.56, -397.65), (323.37, -546.56)]  

# For Tray 2: Screw locations of tray 2 in tray coordinate frame and Robot coordinate frame (in mm)
Screws_wrt_Tray_2 = [(0, 0), (204.3, 0)]
Screws_wrt_RobotBase = [(520.12, -343.21),  (656.34, -193.45)]

# Compute the transformation for each ball (in mm)
# Transformations for the ball positions in the Tray 1
Rot_1, Transl_1, Del_theta_1 = transl_rot(Screws_wrt_Tray_1, Screws_wrt_Robot)
Balls_wrt_RobotFrame = Points_Transformation(Balls_wrt_Tray, Rot_1, Transl_1, Del_theta_1)

# Transformations for the ball positions in the tray 2
Rot_2, Transl_2, Del_theta_2 = transl_rot(Screws_wrt_Tray_2, Screws_wrt_RobotBase)
Robot_Balls_2 = Points_Transformation(Balls_wrt_Tray, Rot_2, Transl_2, Del_theta_2)


# Defining a function to pick and place the balls

def Pick_and_Place(Tray_1_Ball_Pose, Tray_2_Ball_Pose, Ball_Num):
    # Setting some constants
    Retraction_Point = 150  
    Max_Speed = 250     
    Min_Speed = 50      
    Max_Accl = 200     
    Min_Accl = 100     

    # Converting tuple to list so that they can be mutable
    Tray1_Balls = list(Tray_1_Ball_Pose)  
    Balls_Dest = list(Tray_2_Ball_Pose)   
    
    Tray1_Balls[2] += Retraction_Point
    Balls_Dest[2] += Retraction_Point  

    def UR5_Move(Pose, Velocity, Acceleration, Gripper_State):
        robot.setSpeed(Velocity)
        robot.setAcceleration(Acceleration)
        robot.MoveL(xyzrpw_2_pose(Pose))
        move_to(Pose, gripper_state="open" if Gripper_State else "closed")

    # Defining the sequence of picking
    UR5_Move(Tray1_Balls, Max_Speed, Max_Accl, True)
    UR5_Move(Tray_1_Ball_Pose, Min_Speed, Min_Accl, True)
    
    robot.RunCodeCustom('rq_close', INSTRUCTION_CALL_PROGRAM)
    command_gripper("close", force=100)
    robot.RunCodeCustom('sleep(2)', INSTRUCTION_CALL_PROGRAM)
    
    UR5_Move(Tray1_Balls, Min_Speed, Min_Accl, False)

    # Defining the sequence of placing the balls
    UR5_Move(Balls_Dest, Max_Speed, Max_Accl, False)
    UR5_Move(Tray_2_Ball_Pose, Min_Speed, Min_Accl, False)
    
    command_gripper("open")
    robot.RunCodeCustom('rq_open', INSTRUCTION_CALL_PROGRAM)
    robot.RunCodeCustom('sleep(2)', INSTRUCTION_CALL_PROGRAM)
    
    UR5_Move(Balls_Dest, Min_Speed, Min_Accl, True)

# Defining a function to carry out pick and place for particular balls
def Pick_and_Place_Selected_Balls():
    Intermediate_Pose = [235.780, -253.100, 300.00, 180.00, 0, -60]  # Changed to list
    Intermediate_Point = [114.603320, -109.758660, 94.342044, -74.583384, -90.000000, -95.396680]  # Changed to list
    Home = [0, -90, 0, -90, 0, 0]
    
    # Defining the ball number to be picked and placed starting from 0 index
    selected_balls = [1, 3, 4, 6, 9, 11]
    robot.MoveJ(xyzrpw_2_pose(Intermediate_Pose))
    
    for ball in selected_balls:
        ball_num = ball + 1  
        
        Force = 150 if ball_num in {2, 5, 10} else 100
        robot.RunCodeCustom(f'rq_set_force({Force})', INSTRUCTION_CALL_PROGRAM)
        robot.RunCodeCustom('rq_set_speed(100)', INSTRUCTION_CALL_PROGRAM)
        Tray_1_Source_position = list(Balls_wrt_RobotFrame[ball])  
        Target_Pose = list(Robot_Balls_2[ball])                  
        Pick_and_Place(Tray_1_Source_position, Target_Pose, ball_num)
    
    # Return to home position
    robot.MoveJ(Home)

Pick_and_Place_Selected_Balls()