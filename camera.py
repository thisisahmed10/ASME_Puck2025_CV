import cv2
import numpy as np

class CameraVisionModule:
    def __init__(self, CameraNumber):
        self.Camera_Cap = cv2.VideoCapture(CameraNumber)
        _, self.Frame = self.Camera_Cap.read()
        self.Camera_Cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.Camera_Cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.Center_X = self.Frame.shape[1] // 2
        self.Center_Y = self.Frame.shape[0] // 2

    def Read_Frame(self):
        _, self.Frame = self.Camera_Cap.read()

    def Hsv_Frame(self):
        return cv2.cvtColor(self.Frame, cv2.COLOR_BGR2HSV)
    
    def Red_Blue(self,data):
        hsv_img = self.Hsv_Frame()
        pixels = hsv_img.reshape(-1, 3)
        pixels = np.float32(pixels)
        # k-means
        K = 3
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        _, labels, centers = cv2.kmeans(pixels, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

        target_hsv = {'red': np.array([0, 255, 255]), 'blue': np.array([120, 255, 255])}
        if data not in target_hsv:
            raise ValueError('data must be "red" or "blue"')

        distances = [np.linalg.norm(center - target_hsv[data]) for center in centers]
        best_idx = np.argmin(distances)
        center_hsv = centers[best_idx]

        h, s, v = center_hsv
        lower = np.array([max(h-15,0), max(s-60,0), max(v-60,0)], dtype=np.uint8)
        upper = np.array([min(h+15,180), min(s+60,255), min(v+60,255)], dtype=np.uint8)
        mask = cv2.inRange(hsv_img, lower, upper)
        return mask
    
    def Distance_Center_Frame(self, cX_Contour, cY_Contour):
        Dis = []
        Distance = np.sqrt((cX_Contour - self.Center_X) ** 2 + (cY_Contour - self.Center_Y) ** 2)
        Dis.append(Distance)
        Min_Distance = min(Dis)

        if(cX_Contour>self.Center_X):
            return (f'+{Min_Distance}')
        else:
            return (f'-{Min_Distance}')

    def Detect_rectangle(self):
        gray = cv2.cvtColor(self.Frame, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 10, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)
        for cnt in contours:
            x1, y1 = cnt[0][0]
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(cnt)
                ratio = float(w)/h
                print("ok")
                if ratio >= 0.9 and ratio <= 1.1:
                    frame = cv2.drawContours(self.Frame, [cnt], -1, (0,255,255), 3)
                    cv2.putText(frame, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    cv2.putText(frame, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    frame = cv2.drawContours(self.Frame, [cnt], -1, (0,255,0), 3)

    def Detect_Shape(self):
        Color_Mask = self.Red_Blue('blue')
        kernel = np.ones((5,5),np.uint8)
        Mask_kernel = cv2.erode(Color_Mask,kernel)
        contours, _ = cv2.findContours(Mask_kernel,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            if area > 2000:
                cv2.drawContours(self.Frame,[approx],0,(0,255,0),3)
                if 3 < len(approx) < 5:
                    cv2.putText(self.Frame,"Rectangle",(x,y),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,0),cv2.LINE_4)
                elif 8 < len(approx) < 16:
                    cv2.putText(self.Frame,"Circle",(x,y),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,0),cv2.LINE_4)
                    Mm = cv2.moments(cnt)
                    cX = int(Mm['m10']/Mm['m00'])
                    cY = int(Mm['m10']/Mm['m00'])
                    Dis = self.Distance_Center_Frame(cX,cY)
    def Grid_Puck_Count(self):
        h, w = self.Frame.shape[:2]

        height_ratios = [0.25, 0.35, 0.40]
        width_ratios = [0.20, 0.30, 0.30, 0.20]

        height_bounds = [int(sum(height_ratios[:i]) * h) for i in range(4)]
        width_bounds = [int(sum(width_ratios[:i]) * w) for i in range(5)]
        puckSatArr = np.zeros((2, 3, 4), dtype=int)

        for i in range(3):
            for j in range(4):
                y1, y2 = height_bounds[i], height_bounds[i+1]
                x1, x2 = width_bounds[j], width_bounds[j+1]
                roi = self.Frame[y1:y2, x1:x2]
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                # Red and blue mask
                red_mask = self.Red_Blue('red')[y1:y2, x1:x2]
                blue_mask = self.Red_Blue('blue')[y1:y2, x1:x2]
                kernel = np.ones((5,5),np.uint8)
                red_mask = cv2.erode(red_mask, kernel)
                blue_mask = cv2.erode(blue_mask, kernel)
                # red contours
                contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                for cnt in contours_red:
                    area = cv2.contourArea(cnt)
                    approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                    if area > 2000 and 8 < len(approx) < 16:
                        puckSatArr[0, i, j] += 1
                # blue contours
                contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                for cnt in contours_blue:
                    area = cv2.contourArea(cnt)
                    approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                    if area > 2000 and 8 < len(approx) < 16:
                        puckSatArr[1, i, j] += 1
        return puckSatArr

def main(): 
        Cam_obj = CameraVisionModule(0)
        qcd = cv2.QRCodeDetector()
        count = 0
        while True:
            Cam_obj.Read_Frame()
            ret, points, qrcode = qcd.detectAndDecode(Cam_obj.Frame)
            if ret:
                print("seen", count)
                count += 1

            Cam_obj.Detect_rectangle()
            puck_grid = Cam_obj.Grid_Puck_Count()
            print("Grid (R, B):\n", puck_grid)
            cv2.imshow("code_test", Cam_obj.Frame)
            if cv2.waitKey(1) == ord('q'):
                Cam_obj.Camera_Cap.release()
                cv2.destroyAllWindows()
                break
            
if __name__ == "__main__":
    main()