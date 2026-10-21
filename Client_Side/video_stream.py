import cv2

class video_stream:
    def __init__(self):
        self.stream_url = "http://10.42.0.124:8080/video"
        self.cap = None

        self.crop_values = [[0, 1920], [0, 1080]]
        self.resolution = [1080, 1920]

    """SETTINGS"""

    def set_url(self, new_url):
        self.stream_url = new_url

    def set_crop_values(self, y1, y2, x1, x2):
        self.crop_values = [[y1, y2], [x1, x2]]

    def set_resolution(self, x, y):
        self.resolution = [x, y]

    def set_resolution_scale(self, scale: float):
        self.resolution = [
            int(self.resolution[0] * scale),
            int(self.resolution[1] * scale)
        ]

    def set_recommended(self):
        self.set_resolution_scale(1)
        self.set_resolution(780, 1270)
        self.set_crop_values(0, 900, 0, 780)

    """INIT"""

    def initialise_video_stream(self):
        self.cap = cv2.VideoCapture(self.stream_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


    """CAPTURE"""

    def get_frame(self):
        return self.cap.read()

    def get_processed_image(self):
        ret, frame = self.get_frame()
        if not ret or frame is None:
            return ret, None

        resized = cv2.resize(frame, (self.resolution[0], self.resolution[1]), interpolation=cv2.INTER_AREA)


        y1, y2 = self.crop_values[0]
        x1, x2 = self.crop_values[1]
        cropped = resized[y1:y2, x1:x2]

        return ret, cropped

    def get_frame_with_alignment(self):
        ret, frame = self.get_processed_image()

        w, h = self.resolution

        # working area
        cv2.ellipse(frame, (w // 2, 775), (w // 2, 100), 0, 0, 360, (255, 0, 125), 4)

        # arm itself
        cv2.line(frame, (w//2, 670), (w//2, 415), (0, 255, 0), 6)
        cv2.line(frame, (w//2, 260), (w//2, 190), (0, 255, 0), 6)
        cv2.line(frame, (w//2 - 60, 440), (w//2 - 60, 220), (0, 255, 0), 6)


        return ret, frame

    """CONTROL"""

    def save_image(self, path):
        ret, frame = self.get_processed_image()
        cv2.imwrite(path, frame)

    """TEST"""

    def show_frames(self):
        while True:
            ret, frame = self.get_processed_image()

            if not ret:
                break

            cv2.imshow("Video Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    def show_with_alignment(self):
        while True:
            ret, frame = self.get_frame_with_alignment()

            if not ret:
                break

            cv2.imshow("Video Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    """EXIT"""

    def stop_stream(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    print("--------INFO--------")
    print("Abstand Vorderdisplay <-> Unteres Roboterarmsegment: ca. 76,5cm")
    print("--------------------")

    vs = video_stream()
    vs.set_url("http://10.42.0.124:8080/video")
    vs.initialise_video_stream()
    vs.set_recommended()
    vs.show_with_alignment()
