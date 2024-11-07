import cv2
import pickle

# Throw and error if the camera wasn't successfully opened
class videoCaptureError(Exception):
    pass

class MyCamera():
    def __init__(self, resolution, videoSource = 0):

        # Open the video source
        self.cap = cv2.VideoCapture(videoSource)

        # Check if the camera was successfully opened
        if self.cap.isOpened():
            print('La camára fue detectada satisfactoriamente.')
        else:
            raise videoCaptureError('No se pudo abrir la cámara (fijarse si se conectó correctamente o probar otro valor para videoSource).')
        
        # Establish the desired resolution and get the camera parameters
        if resolution == '480p':
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            # Load the data from the pickle file
            with open('../config/camera_config_480p.pckl', 'rb') as file:
                loadedData = pickle.load(file)

            # Default values obtained through calibration are specific to the Q-BOX camera
            self.cameraMatrix = loadedData[0]
            self.distCoeffs = loadedData[1]

        elif resolution == '1080p':
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

            # Load the data from the pickle file
            with open('../config/camera_config_1080p.pckl', 'rb') as file:
                loadedData = pickle.load(file)

            # Default values obtained through calibration are specific to the Q-BOX camera
            self.cameraMatrix = loadedData[0]
            self.distCoeffs = loadedData[1]

        # Discard initial frames to allow the camera to adjust itself (particular problem with our camera)
        for _ in range(50):
            self.cap.read()

        # Get video width and height
        self.frameWidth = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frameHeight = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def get_frame(self):
        if self.cap.isOpened():
            success, frame = self.cap.read()

            # if the frame was read correctly
            if success:
                return frame
            else:
                return None
        else:
            return None
        
    def release_camera(self):
        if self.cap.isOpened():
            self.cap.release()
            cv2.destroyAllWindows()
    
    # If you have already obtained the frame and just need to display it
    @staticmethod
    def show_frame(frame):
        cv2.imshow('Frame', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == '__main__':

    cap1 = MyCamera('480p', 0)

    frame1 = cap1.get_frame()

    print(cap1.frameHeight)
    print(cap1.frameWidth)

    MyCamera.show_frame(frame1)

    cap1.release_camera()