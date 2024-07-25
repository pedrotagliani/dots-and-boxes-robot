import numpy as np

def simple_interpolator(initialPoint, finalPoint, lineType):

    # Number of points to interpolate
    numPoints = 10

    if lineType == 'Vertical':
        # Interpolating x values is necessary, while y and z values remain constant

        xValues = np.linspace(initialPoint[0], finalPoint[0], numPoints)

        yValues = np.full(numPoints, initialPoint[1])

        zValues = np.full(numPoints, initialPoint[2])

        # Combine the values into an array of points
        interpolatedPoints = np.vstack((xValues, yValues, zValues)).T


    elif lineType == 'Horizontal':
        # Interpolating y values is necessary, while x and z values remain constant
        
        yValues = np.linspace(initialPoint[1], finalPoint[1], numPoints)

        xValues = np.full(numPoints, initialPoint[0])

        zValues = np.full(numPoints, initialPoint[2])

        # Combine the values into an array of points
        interpolatedPoints = np.vstack((xValues, yValues, zValues)).T
    
    return interpolatedPoints


if __name__ == '__main__':
    
    # Define the line
    line = 'Horizontal'
    initialPoint = [37, 6.2, 0.2]
    finalPoint = [37, 9.8, 0.2]

    interpolatedPoints = simple_interpolator(initialPoint, finalPoint, line)

    print(interpolatedPoints)
