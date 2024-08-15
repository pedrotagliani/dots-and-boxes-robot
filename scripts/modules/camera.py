import cv2

# Throw and error if the camera wasn't successfully opened
class videoCaptureError(Exception):
    pass

class myCamera():
    def __init__(self, videoSource = 0):

        # Open the video source
        self.cap = cv2.VideoCapture(videoSource)

        # Check if the camera was successfully opened
        if self.cap.isOpened():
            print('La camára fue detectada satisfactoriamente.')
        else:
            raise videoCaptureError('No se pudo abrir la cámara (fijarse si se conectó correctamente o probar otro valor para videoSource).')
        
        # Discard initial frames to allow the camera to adjust itself (particular problem with our camera)
        for _ in range(50):
            self.cap.read()

        # Get video width and height
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

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
    
    # If you have already obtained the frame and just need to display it
    @staticmethod
    def show_frame(frame):
        cv2.imshow('Frame', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == '__main__':

    cap1 = myCamera(0)

    frame1 = cap1.get_frame()

    print(cap1.height)
    print(cap1.width)

    myCamera.show_frame(frame1)