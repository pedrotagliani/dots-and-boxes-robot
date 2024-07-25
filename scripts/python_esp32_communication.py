from interpolator import simple_interpolator
from inverse_kinematics import inverse_kinematics
import serial
import time

def send_angles(serial_port, angles):
    # Send the angles to the ESP32
    for angle in angles:
        serial_port.write(f"{angle}\n".encode())
    serial_port.flush()

def send_home_angles(serial_port):
    homeAngles = [90, 90, 0, 120, 1]
    print('Volviendo a la posición de home...')

    for angle in homeAngles:
        serial_port.write(f"{angle}\n".encode())
    serial_port.flush()

def serial_communication_for_servos(interpolatedPoints):
    # Create the serial communication object
    ser = serial.Serial('COM6', 115200)
    time.sleep(4)  # Wait for the connection to establish

    print('Comenzó la trayectoria.')

    for point in interpolatedPoints:

        # Get the angles from the inverse kinematics
        angles = inverse_kinematics(point[0], point[1], point[2], 0)

        angles.append(0)

        # Use the function to send the angles to the ESP32
        send_angles(ser, angles)

        while True:
            if ser.in_waiting: # Check if there is data available in the input buffer
                response = ser.readline().decode().strip()
                if response == "DONE":
                    print("Movimiento realizado, mandando los siguientes ángulos...")
                    break
        
    send_home_angles(ser)

    while True:
        if ser.in_waiting: # Check if there is data available in the input buffer
            response = ser.readline().decode().strip()
            if response == "DONE":
                print("El brazo volvió a la posición de home.")
                break

    ser.close()

if __name__ == '__main__':
    # Define the line
    line = 'Horizontal'
    initialPoint = [37, 6.2, 0.2]
    finalPoint = [37, 9.8, 0.2]

    interpolatedPointsArray = simple_interpolator(initialPoint, finalPoint, line)

    serial_communication_for_servos(interpolatedPointsArray)
