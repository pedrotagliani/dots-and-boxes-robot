import serial
import time
from interpolation import position_quintic_interpolation
from inverse_kinematics import inverse_kinematics
from forward_kinematics import forward_kinematics
from math import radians

# Define the home joint poosition of the robot
homeJointAngles = [90.0, 80.0, 0.0, 120.0]

# Define the position [x, y, z, pitchAngle] in the home position
homePosition = [20.47, 0, 27.71, -74.0]

# The encoders' placement isn't set exactly as it should be, so the measured obtained from them won't be so precise
# Hence, it is required to set a margin error to compare them with another angles
# Unit = [deg]
marginError = 5 

# For the moment, I'll assume all the responses from the robot are the correct ones

def start_communication_with_robot(port):
    # Create the serial communication object
    ser = serial.Serial(port, 115200)
    time.sleep(4)  # Wait for the connection to establish

    return ser

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
    
    print(qValues)

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




def send_angles(ser, interpolatedPoints):

    print('Iniciando movimiento...')

    # Send a message to tell the robot a movement action is coming
    ser.write("b\n".encode("utf-8"))

    # Make sure the robot understood the choice
    while True:
        if ser.in_waiting > 0:  # Check if there's incoming data
            response = ser.readline().decode().strip()  # Read the line, decode it to string, and strip whitespace
            
            if response == "sendit":
                break
                # Ok, we can start to send the angles
            else:
                print("El robot no entendió la opción seleccionada.")

    interpolatedPoints = interpolatedPoints[1:,:]

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


def make_robot_play(initialPoint, finalPoint):

    # Establish the serial communication between the robot and the ESP32
    ser = start_communication_with_robot('COM5')

    # Check if the robot is placed in the home position to start drawing
    isHome = is_robot_home(ser)

    if isHome:
        print("El robot está en la posición de home. Está listo para hacer su movimiento.")
        
        # We need to generate a trajectory to go from the home position to the initial point
        interpolatedPointsHtoI = position_quintic_interpolation(homePosition, initialPoint)

        # interpolatedPoints = [x, y, z, pitchAngle]

        # It's time to convert the points to jont positions and then send them to the robotic arm actuators
        send_angles(ser, interpolatedPointsHtoI)

        # Generate the trajectory to go from the initial point to the final point (game movement)
        interpolatedPointsItoF = position_quintic_interpolation(initialPoint, finalPoint)
        
        # Send them to the robot
        send_angles(ser, interpolatedPointsItoF)

        # Generate the trajectory to go back to the home position
        interpolatedPointsFtoH = position_quintic_interpolation(finalPoint, homePosition)

        # Send them to the robot
        send_angles(ser, interpolatedPointsFtoH)

        print("Se completó el movimiento del robot.")
        
        # Close serial port
        ser.close()
    
    else:
        print("El robot no está en la posición de home. Tiene que reiniciar el juego y acomodar el brazo.")


def move_robot_to(fromPoint, toPoint):
    
    # Establish the serial communication between the robot and the ESP32
    ser = start_communication_with_robot('COM5')

    interpolatedPointsHtoI = position_quintic_interpolation(fromPoint, toPoint)

    send_angles(ser, interpolatedPointsHtoI)

    print("Se completó el movimiento del robot.")
    
    # Close serial port
    ser.close()



if __name__ == '__main__':

    x0 = 23.4
    y0 = -5.4
    z0 = 0.5
    pitchAngle0 = 0

    x1 = 23.4 + 3.6
    y1 = -5.4
    z1 = 0.2
    pitchAngle1 = 0


    p0 = [x0, y0, z0, pitchAngle0]
    
    p1 = [x1, y1, z1, pitchAngle1]

    make_robot_play(p0, p1)