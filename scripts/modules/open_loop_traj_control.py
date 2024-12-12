import serial
import time

# from interpolation import position_quintic_interpolation
# from inverse_kinematics import inverse_kinematics
# from forward_kinematics import forward_kinematics

from modules.interpolation import position_quintic_interpolation
from modules.inverse_kinematics import inverse_kinematics
from modules.forward_kinematics import forward_kinematics

from math import radians
import numpy as np

# Define the home joint poosition of the robot
homeJointAngles = [90.0, 80.0, 0.0, 120.0]

# Define the position [x, y, z, pitchAngle] in the home position
homePosition = [19.51, 0, 27.99, -74.0]

# The encoders' placement isn't set exactly as it should be, so the measured obtained from them won't be so precise
# Hence, it is required to set a margin error to compare them with another angles
# Unit = [deg]
marginError = 10

# For the moment, I'll assume all the responses from the robot are the correct ones

def start_communication_with_robot(port):
    # Create the serial communication object
    ser = serial.Serial(port, 115200)
    # time.sleep(4)  # Wait for the connection to establish

    return ser

def is_robot_awake(ser):

    # Ask the robot if it is turned on by choosing option "e"
    ser.write(f"{"e"}\n".encode("utf-8"))

    # Verify that the robot is turned on
    while True:
        if ser.in_waiting > 0:  # Check if there's incoming data
            response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
            print(response)

            
            if response == "on":
                break
                # Ok, it's turned on 
            else:
                print("El robot está apagado o no responde.")
                quit()

def read_encoders(ser):
    
    # Send a message to ask for the encoders
    ser.write("a\n".encode("utf-8"))

    while True:
        # Read a line from the serial connection
        if ser.in_waiting > 0:  # Check if there's incoming data
            # serial_port.in_waiting -----------> Indicate the number of bytes currently available in the input buffer
            response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
            qValues = list(map(float, response.split(',')))
            break
    
    print(f"Lectura de encoders: {qValues}")

    return qValues

def is_robot_home(ser):
    # Get the current angles
    currentJointAngles = read_encoders(ser)

    # We obtained the current angles, now we need to check if they correspond to the home position
    boolCheckq1 = (currentJointAngles[0] <= homeJointAngles[0] + marginError) and (currentJointAngles[0] >= homeJointAngles[0] - marginError)
    boolCheckq2 = (currentJointAngles[1] <= homeJointAngles[1] + marginError) and (currentJointAngles[1] >= homeJointAngles[1] - marginError)
    boolCheckq3 = (currentJointAngles[2] <= homeJointAngles[2] + marginError) and (currentJointAngles[2] >= homeJointAngles[2] - marginError)
    boolCheckq4 = (currentJointAngles[3] <= homeJointAngles[3] + marginError) and (currentJointAngles[3] >= homeJointAngles[3] - marginError)

    # If all the values are within the magin error, then return true. Otherwise, false
    if (boolCheckq1 and boolCheckq2 and boolCheckq3 and boolCheckq4):
        return True
    else:
        return False




def send_angles(ser, interpolatedPoints, option):

    print('Iniciando movimiento...')

    # Option "b" ---> HtoI ---> The motors from the base, shoulder and elbow moves synchronized, and after they arrive, the gripper moves
    # It also checks the position of the stepper (90°)
    # Option "c" ---> ItoF ---> All motors move synchronized
    # Option "d" ---> FtoH ---> Gripper moves back first and, after it arrives, the remaining motors move back synchronized

    # Send a message to tell the robot a movement action is coming
    ser.write(f"{option}\n".encode("utf-8"))

    # Make sure the robot understood the choice
    while True:
        if ser.in_waiting > 0:  # Check if there's incoming data
            response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
            
            if response == "ok":
                break
                # Ok, we can start to send the angles
            else:
                print("El robot no entendió la opción seleccionada.")

    if (option == "b" or option == "c" or option == "g"):

        # No interpolation, just the initial point (beginning of the line)
        x = interpolatedPoints[0]
        y = interpolatedPoints[1]
        z = interpolatedPoints[2]
        pitchAngle = radians(interpolatedPoints[3])

        # print([x,y,z,pitchAngle])

        # Compute the inverse kinematics to obtain the joint angles
        # [x, y, z, pitchAngle] to [q1, q2, q3, q4]
        desiredJointAngles = inverse_kinematics(x, y, z, pitchAngle)

        print(desiredJointAngles)

        # Send the calculated angles to the robot
        anglesToSend = ",".join(map(str, desiredJointAngles)) + "\n"
        ser.write(anglesToSend.encode('utf-8'))

        # It's needed to wait till the robot confirms the movement has been performed
        while True:
            # Read a line from the serial connection
            if ser.in_waiting > 0:  # Check if there's incoming data
                response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
                if response == "done":
                    break
                else:
                    print('El robot no llegó a destino. Reiniciar la partida.')
        
        # Send a message to notify the robot that the trajectory is complete
        ser.write("completed\n".encode("utf-8"))
    
    elif (option == "d" or option == "h"):

        # It's needed to wait till the robot confirms the movement has been performed
        while True:
            # Read a line from the serial connection
            if ser.in_waiting > 0:  # Check if there's incoming data
                response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace

                if response == "done":
                    break
                else:
                    print('El robot no llegó a destino. Reiniciar la partida.')
    
    elif option == "f":

        for point in interpolatedPoints:

            x = point[0]
            y = point[1]
            z = point[2]
            pitchAngle = radians(point[3])

            # print([x,y,z,pitchAngle])

            # Compute the inverse kinematics to obtain the joint angles
            # [x, y, z, pitchAngle] to [q1, q2, q3, q4]
            desiredJointAngles = inverse_kinematics(x, y, z, pitchAngle)

            print(desiredJointAngles)

            # Send the calculated angles to the robot
            anglesToSend = ",".join(map(str, desiredJointAngles)) + "\n"
            ser.write(anglesToSend.encode('utf-8'))

            # It's needed to wait till the robot confirms the movement has been performed
            while True:
                # Read a line from the serial connection
                if ser.in_waiting > 0:  # Check if there's incoming data
                    response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace

                    if response == "more":
                        break
                        # Now, we can keep sending the remaining angles
                    else:
                        print('El robot no llegó a destino. Reiniciar la partida.')

        # Send a message to notify the robot that the trajectory is complete
        ser.write("completed\n".encode("utf-8"))

    print('Movimiento finalizado.')
    qval = read_encoders(ser)
    


def make_robot_play(initialPoint, finalPoint):

    # Establish the serial communication between the robot and the ESP32
    ser = start_communication_with_robot('COM5')

    # Check if the robot has been turned on
    is_robot_awake(ser)

    # Check if the robot is placed in the home position to start drawing
    isHome = is_robot_home(ser)

    if isHome:
        print("El robot está en la posición de home. Está listo para hacer su movimiento.")
        
        # interpolatedPoints = [x, y, z, pitchAngle]

        # It's time to convert the points to jont positions and then send them to the robotic arm actuators
        # We won't interpolate betweeen this points using the created function
        send_angles(ser, initialPoint, "b")

        # Generate the trajectory to go from the initial point to the final point (game movement)
        interpolatedPointsItoF = position_quintic_interpolation(initialPoint, finalPoint,70)
        
        # Send them to the robot
        # send_angles(ser, finalPoint, "c")
        send_angles(ser, finalPoint, "g")

        # It's time to convert the points to jont positions and then send them to the robotic arm actuators
        # We won't interpolate betweeen this points using the created function
        send_angles(ser, homePosition, "d")

        print("Se completó el movimiento del robot.")
        
        # Close serial port
        ser.close()
    
    else:
        print("El robot no está en la posición de home. Tiene que reiniciar el juego y acomodar el brazo.")


def move_robot_to(fromPoint, toPoint):
    
    # Establish the serial communication between the robot and the ESP32
    ser = start_communication_with_robot('COM4')

    interpolatedPointsHtoI = position_quintic_interpolation(fromPoint, toPoint)

    send_angles(ser, interpolatedPointsHtoI)

    print("Se completó el movimiento del robot.")
    
    # Close serial port
    ser.close()



if __name__ == '__main__':

    # z = 8.8
    # pitchAngle = 8.0

    # gameArray = np.array(
    #     # First row
    #     [[[23.8 - 0.6, -5.4 - 0.1,  z - 0.5, pitchAngle],
    #     [23.4 - 0.1, -1.8,  z - 0.6, pitchAngle],
    #     [23.4 + 0.4,  1.8 - 0.1,  z - 0.4, pitchAngle],
    #     [23.4 - 0.1,  5.4 - 0.1, z - 0.6, pitchAngle]],

    #     # Second row
    #     [[27.0 - 1.2, -5.4 + 0.1,  z - 1.1, pitchAngle],
    #     [27.0 - 0.9, -1.8 + 0.1,  z - 1.1, pitchAngle],
    #     [27.0 - 0.7,  1.8 + 0.1,  z - 1.2, pitchAngle],
    #     [27.0 - 0.9,  5.4 - 0.1,  z - 1.3, pitchAngle]],

    #     # Third row
    #     [[30.6 - 1.5, -5.4 + 0.3,  z - 0.8, pitchAngle],
    #     [30.6 - 1.1, -1.8 + 0.2,  z - 0.7, pitchAngle],
    #             # [30.6 - 1.1, -1.8 + 0.2,  z - 0.8, pitchAngle],
    #     [30.6 - 1.0,  1.8 + 0.1,  z - 0.7, pitchAngle],
    #                     # [30.6 - 1.0,  1.8 + 0.1,  z - 0.8, pitchAngle],
    #     [30.6 - 1.5,  5.4,  z - 1.1, pitchAngle]]]
    # )

    z = 6.4
    pitchAngle = 8.0

    gameArray = np.array(
        # First row
        [[[23.8 - 0.3, -5.4 + 0.4,  z + 0.6, pitchAngle],
        # [[[23.8 - 0.2, -5.4 + 0.4,  z + 0.6, pitchAngle],
        [23.4 + 0.4, -1.8 + 0.4,  z + 0.6, pitchAngle],
        [23.4 + 0.6,  1.8 + 0.8,  z + 0.6, pitchAngle],
        # [23.4 + 0.6,  1.8 + 0.6,  z + 0.6, pitchAngle],
        [23.4 + 0.2,  5.4 + 0.7, z + 0.6, pitchAngle]],

        # Second row
        [[27.0 - 0.5, -5.4 + 0.4,  z + 0.3, pitchAngle],
        [27.0 - 0.3, -1.8 + 0.5,  z + 0.2, pitchAngle],
        [27.0 - 0.1,  1.8 + 0.5,  z + 0.3, pitchAngle],
        [27.0 - 0.6,  5.4 + 0.6,  z + 0.3, pitchAngle]],

        # Third row
        [[30.6 - 1.5, -5.4 + 0.8,  z - 0.3, pitchAngle],
        [30.6 - 1.4, -1.8 + 1,  z - 0.3, pitchAngle],
        [30.6 - 1.2,  1.8 + 0.9,  z - 0.3, pitchAngle],
        [30.6 - 1.5,  5.4 + 0.6,  z - 0.3, pitchAngle]]]
    )


    p0 = list(gameArray[0][0])

    p1 = list(gameArray[1][0])
 
    make_robot_play(p0, p1)

    # print(inverse_kinematics(p0[0], p0[1], p0[2], radians(p0[3])))
    # print(inverse_kinematics(p1[0], p1[1], p1[2], radians(p1[3])))

    horizontalLinesArray = np.array([[[[0.0, 0.0],
                                    [0.0, 1.0]],

                                    [[0.0, 1.0],
                                    [0.0, 2.0]],

                                    [[0.0, 2.0],
                                    [0.0, 3.0]]],


                                    [[[1.0, 0.0],
                                    [1.0, 1.0]],

                                    [[1.0, 1.0],
                                    [1.0, 2.0]],

                                    [[1.0, 2.0],
                                    [1.0, 3.0]]],


                                    [[[2.0, 0.0],
                                    [2.0, 1.0]],

                                    [[2.0, 1.0],
                                    [2.0, 2.0]],

                                    [[2.0, 2.0],
                                    [2.0, 3.0]]]])

    verticalLinesArray = np.array([[[[0.0, 0.0],
                                    [1.0, 0.0]],

                                    [[0.0, 1.0],
                                    [1.0, 1.0]],

                                    [[0.0, 2.0],
                                    [1.0, 2.0]],

                                    [[0.0, 3.0],
                                    [1.0, 3.0]]],


                                    [[[1.0, 0.0],
                                    [2.0, 0.0]],

                                    [[1.0, 1.0],
                                    [2.0, 1.0]],

                                    [[1.0, 2.0],
                                    [2.0, 2.0]],

                                    [[1.0, 3.0],
                                    [2.0, 3.0]]]])