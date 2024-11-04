from inverse_kinematics import inverse_kinematics
from forward_kinematics import forward_kinematics
import roboticstoolbox as rtb
from spatialmath import SE3, base, SO3
import matplotlib.pyplot as plt
from math import pi, degrees, radians
import numpy as np
from spatialmath.base import tr2rpy
from mpl_toolkits.mplot3d import Axes3D
import spatialgeometry as sg
import time
import math
import serial

# Home angles
q1Home = 90
q2Home = 80
q3Home = 0
q4Home = 120

l1 = 14.085
l2 = 12.725
l3 = 10.222
l4 = 11.200
l5 = 8.400

qHome = [radians(q1Home), radians(q2Home), radians(q3Home), radians(q4Home)]

# Robot's links
link1 = rtb.RevoluteDH(d=l1, a=0.0, alpha=pi/2, offset=-pi/2, qlim=[0,pi])
link2 = rtb.RevoluteDH(d=0.0, a=l2, alpha=0.0, offset=0.0, qlim=[0,pi])
link3 = rtb.RevoluteDH(d=0.0, a=l3, alpha=0.0, offset=-radians(126), qlim=[0,pi])
link4 = rtb.RevoluteDH(d=0.0, a=l4, alpha=0.0, offset=0.0, qlim=[0,pi])

# Create the robot
robot = rtb.DHRobot(
    links=[link1, link2, link3, link4],
    name="D&B Robot",
    tool = SE3.Ty(-l5)
    )

# Get the Homogeneous Transformation Matrix (HTM) for the home position relative to the base frame
THome = forward_kinematics(qHome)

def wait_till_home(serial_port):

    while True:
        # Read a line from the serial connection
        if serial_port.in_waiting > 0:  # Check if there's incoming data
            # serial_port.in_waiting -----------> Indicate the number of bytes currently available in the input buffer
            response = serial_port.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace

            # Check if the message is "home"
            if response == "home":
                print('El robot está en home.')
                break

# type(T0) and type(T1) ----> spatialmath.pose3d.SE3
def trajectory_generation(T0, T1):
    
    # Set time step in seconds
    dt = 0.2

    # Total time in seconds
    totalTime = 8

    # Time array
    trajTime = np.arange(0, totalTime, dt)

    # Number of steps
    stepsNum = len(trajTime)

    # https://petercorke.github.io/robotics-toolbox-python/arm_trajectory.html#roboticstoolbox.tools.trajectory.trapezoidal
    # Generate the trapezoidal trajectory for the translational components
    x = rtb.tools.trajectory.trapezoidal(T0.A[0,3], T1.A[0,3], trajTime)
    y = rtb.tools.trajectory.trapezoidal(T0.A[1,3], T1.A[1,3], trajTime)
    z = rtb.tools.trajectory.trapezoidal(T0.A[2,3], T1.A[2,3], trajTime)
    # Unit = [cm]

    # Velocities of the translational components along the trajectory (linear velocities)
    vx = x.qd
    vy = y.qd
    vz = z.qd
    # Unit = [cm/s]

    # Transform the initial and final rotation matrices to angle-axis representation
    angAxisT0 = rotation_matrix_to_angle_axis_repr(T0.R)
    angAxisT1 = rotation_matrix_to_angle_axis_repr(T1.R)

    # Generate the trapezoidal trajectory for the orientation components in angle-axis representation form
    angx = rtb.tools.trajectory.trapezoidal(angAxisT0[0], angAxisT1[0], trajTime)
    angy = rtb.tools.trajectory.trapezoidal(angAxisT0[1], angAxisT1[1], trajTime)
    angz = rtb.tools.trajectory.trapezoidal(angAxisT0[2], angAxisT1[2], trajTime)
    angs = [np.array([angx.q[i], angy.q[i], angz.q[i]])for i in range(len(angx))]
    # Unit = [rad]

    # Get the angular velocities
    wx = angx.qd
    wy = angy.qd
    wz = angz.qd
    # Unit = [rad/s]

    # Transform the trajectory represented in angle-axis form to rotation matrix
    rotMatrix = [(SO3.AngleAxis(np.linalg.norm(angs[i]),angs[i]/np.linalg.norm(angs[i]))).R for i in range(len(trajTime))]

    # Create the homogeneous transformation matrices using the rotation matrices and translation vectors from the trajectory
    XRef = [SE3.Rt(rotMatrix[j], np.array([x.q[j], y.q[j], z.q[j]])) for j in range(len(trajTime))]

    # Organize the velocities to ease its manipulation
    XdRef = [np.array([vx[i], vy[i], vz[i], wx[i], wy[i], wz[i]]) for i in range(len(trajTime))]

    return XRef, XdRef, dt

def rotation_matrix_to_angle_axis_repr(R):
    
    # Transform the rotation matrix to its Euler vector equivalent
    
    li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if np.linalg.norm(li) < 1e-6:
        # If li is a zero vector (or very close to it)

        # Diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # Non-diagonal matrix case
        ln = np.linalg.norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    return a

def pose_error(currentPose, desiredPose):

    # Returns the error between the desired and the current pose (position: vector, orientation: angle-axis notation)
    # currentPose and desiredPose --> np.ndarray with shape = (4,4)
    # currentPose and desiredPose are the poses of the end effector relative to the base frame
    
    if isinstance(currentPose, SE3):
        currentPose = currentPose.A

    if isinstance(desiredPose, SE3):
        desiredPose = desiredPose.A

    # Error array
    e = np.empty(6)

    # The position error
    e[:3] = desiredPose[:3, -1] - currentPose[:3, -1]

    # The rotation error expressed as rotation matrix
    R = desiredPose[:3, :3] @ currentPose[:3, :3].T

    # Transform the rotation matrix to its Euler vector equivalent
    a = rotation_matrix_to_angle_axis_repr(R)
    
    # Append the rotation error expressed in the angle-axis representation in the error array
    e[3:] = a

    # Return the error
    # e.shape = (6,)
    return e

def get_current_pose(serial_port):
    while True:
        if serial_port.in_waiting > 0:
            q_data = serial_port.readline().decode('utf-8').strip()
            q_values = list(map(float, q_data.split(',')))
            break
    
    print(q_values)

    T = forward_kinematics(q_values)

    return T, q_values

def send_q_qd(q,qd, serial_port):
    q_data = ",".join(map(str, q)) + "\n"
    serial_port.write(q_data.encode('utf-8'))

    qd_data = ",".join(map(str, qd)) + "\n"
    serial_port.write(qd_data.encode('utf-8'))



def rrmc(XRef, XdRef, dt, serial_port):

    arrived = False

    # Proportional control gain
    gain = 4

    # Construct our gain diagonal matrix
    if base.isscalar(gain):
        kp = gain * np.eye(6)
    else:
        kp = np.diag(gain)
        
    threshold = 1

    i = 0 # Auxiliar variable

    serial_port.write(b"comenzar\n")

    while not arrived:
        
        # start = time.time()
        
        # Current robot's pose
        currentPose, qNow = get_current_pose(serial_port)

        if i < len(XRef):
            # Desired pose  in this specific time step:
            desiredPose = XRef[i]
            actualXdRef = XdRef[i]
        else:
            # If all trajectory points have been processed, ensure the robot's final pose is achieved correctly
            desiredPose = XRef[-1]
            actualXdRef = XdRef[-1]
            
        # Compute the position error
        poseError = pose_error(currentPose, desiredPose)
        
        # Compute the combined term (feedforward + feedback)
        referenceVelocityAdjusted = actualXdRef + (kp @ poseError)
        
        # Use this term to calculate joint velocities using the geometric jacobian relative to the base frame
        qd = np.linalg.pinv(robot.jacob0([radians(qNow[0]), radians(qNow[1]), radians(qNow[2]), radians(qNow[3])])) @ referenceVelocityAdjusted
        
        qd = [degrees(qd[0]), degrees(qd[1]), degrees(qd[2]), degrees(qd[3])]
        q = [qNow[0]+dt*qd[0], qNow[1]+dt*qd[1], qNow[2]+dt*qd[2], qNow[3]+dt*qd[3]]

        print(q)
        print(qd)

        # Apply the calculated joint velocity to the robot's actuators for movement
        send_q_qd(q,qd, serial_port)
        
        if i >= len(XRef):
            # Check if the robotic arm has reached the desired point
            arrived = True if np.sum(np.abs(poseError)) < threshold else False

        # Update the auxiliar variable
        i += 1

        # stop = time.time()
        
        # Complete the time step
        # if stop - start < dt:
        #     time.sleep(dt - (stop - start))


# Movements: Home position (THome) ------> initialPose (T0) ------> finalPose (T1) -------> Home position (THome)
def draw_line_communication(initialPose, finalPose):
    
    # Create the serial communication object
    ser = serial.Serial('COM9', 115200)
    time.sleep(4)  # Wait for the connection to establish

    # Wait till the robot arrives home position
    wait_till_home(ser)

    # Get the desired pose
    pxT0 = initialPose[0]
    pyT0 = initialPose[1]
    pzT0 = initialPose[2]
    pitchAngleT0 = radians(initialPose[3])

    # Its orientation needs to be determined; it's only known that the pitch angle, in the RPY convention, is equal to zero (parallel to the surface)
    qT0 = inverse_kinematics(pxT0, pyT0, pzT0, pitchAngleT0)
    T0 = forward_kinematics(qT0)

    # Get the desired pose
    pxT1 = finalPose[0]
    pyT1 = finalPose[1]
    pzT1 = finalPose[2]
    pitchAngleT1 = radians(finalPose[3])

    # Its orientation needs to be determined; it's only known that the pitch angle, in the RPY convention, is equal to zero (parallel to the surface)
    qT1 = inverse_kinematics(pxT1, pyT1, pzT1, pitchAngleT1)
    T1 = forward_kinematics(qT1)

    # Generate the trajectory to go from THome to T0
    Xref, XdRef, dt = trajectory_generation(THome, T0)

    # Resolved-Rate Motion Control
    rrmc(Xref, XdRef, dt, ser)






















if __name__ == '__main__':

    # Pose ---> [px, py, pz, pitchAngle]
    initialPose1 = [31.67, 0, 13.6, -4.0]
    finalPose1 = [31.67, 0, 13.6, -4.0]

    draw_line_communication(initialPose1, finalPose1)