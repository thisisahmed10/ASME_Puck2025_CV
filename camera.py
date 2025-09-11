import cv2
import numpy as np
import time

class CameraVisionModule:
    def __init__(self, CameraNumber):
        self.Camera_Cap = cv2.VideoCapture(CameraNumber)
        _, self.Frame = self.Camera_Cap.read()
        self.Camera_Cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.Camera_Cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.Center_X = self.Frame.shape[1] // 2
        self.Center_Y = self.Frame.shape[0] // 2
        self._last_calib_time = {'red': 0, 'blue': 0}
        self._cached_mask = {'red': None, 'blue': None}
        self.hsv_thresholds = {
            'red': [(0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)],
            'blue': [(100, 120, 70), (140, 255, 255)]
        }
        self.hsv_thresholds = {
            'red': [(0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)],
            'blue': [(100, 120, 70), (140, 255, 255)]
        }

    def Read_Frame(self):
        _, self.Frame = self.Camera_Cap.read()

    def auto_calibrate(self):
        hsv_img = self.Hsv_Frame()
        pixels = hsv_img.reshape(-1, 3)
        pixels = np.float32(pixels)
        K = 3
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        _, labels, centers = cv2.kmeans(pixels, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        # Closest cluster
        target_hsv = {'red': np.array([0, 255, 255]), 'blue': np.array([120, 255, 255])}
        for color in ['red', 'blue']:
            distances = [np.linalg.norm(center - target_hsv[color]) for center in centers]
            best_idx = np.argmin(distances)
            center_hsv = centers[best_idx]
            h, s, v = center_hsv
            if color == 'red':
                self.hsv_thresholds['red'] = [
                    (max(0, int(h)-10), max(0, int(s)-60), max(0, int(v)-60)),
                    (min(10, int(h)+10), min(255, int(s)+60), min(255, int(v)+60)),
                    (max(170, int(h)-10), max(0, int(s)-60), max(0, int(v)-60)),
                    (min(180, int(h)+10), min(255, int(s)+60), min(255, int(v)+60))
                ]
            else:
                self.hsv_thresholds['blue'] = [
                    (max(100, int(h)-20), max(0, int(s)-60), max(0, int(v)-60)),
                    (min(140, int(h)+20), min(255, int(s)+60), min(255, int(v)+60))
                ]
            #print(f"Auto-calibrated HSV for {color}: {self.hsv_thresholds[color]}")
    def Hsv_Frame(self):
        return cv2.cvtColor(self.Frame, cv2.COLOR_BGR2HSV)
    def Red_Blue(self, color):
        hsv = self.Hsv_Frame()
        if color == 'red':
            t = self.hsv_thresholds['red']
            lower1 = np.array(t[0])
            upper1 = np.array(t[1])
            lower2 = np.array(t[2])
            upper2 = np.array(t[3])
            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = mask1 | mask2
        elif color == 'blue':
            t = self.hsv_thresholds['blue']
            lower = np.array(t[0])
            upper = np.array(t[1])
            mask = cv2.inRange(hsv, lower, upper)
        else:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        return mask

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
                #print("ok")
                if ratio >= 0.9 and ratio <= 1.1:
                    cv2.drawContours(self.Frame, [cnt], -1, (0,255,255), 3)
                    cv2.putText(self.Frame, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    cv2.putText(self.Frame, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.drawContours(self.Frame, [cnt], -1, (0,255,0), 3)

    def Detect_Shape(self, color='blue'):
        mask = self.Red_Blue(color)
        shapes = self.detect_masked_shapes(mask, color)
        return shapes

    def Grid_Puck_Count(self):
        h, w = self.Frame.shape[:2]
        height_ratios = [0.25, 0.35, 0.40]
        width_ratios = [0.20, 0.30, 0.30, 0.20]
        height_bounds = [int(sum(height_ratios[:i]) * h) for i in range(4)]
        width_bounds = [int(sum(width_ratios[:i]) * w) for i in range(5)]
        puckSatArr = np.zeros((2, 3, 4), dtype=int)
        hsv_ranges = {
            'red': [([0,120,70], [10,255,255]), ([170,120,70], [180,255,255])],
            'blue': [([100,120,70], [140,255,255])]
        }
        kernel = np.ones((5,5),np.uint8)
        for color_idx, color in enumerate(['red', 'blue']):
            for i in range(3):
                for j in range(4):
                    y1, y2 = height_bounds[i], height_bounds[i+1]
                    x1, x2 = width_bounds[j], width_bounds[j+1]
                    roi_hsv = cv2.cvtColor(self.Frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
                    mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
                    for lower, upper in hsv_ranges[color]:
                        mask |= cv2.inRange(roi_hsv, np.array(lower), np.array(upper))
                    mask = cv2.erode(mask, kernel)
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    for cnt in contours:
                        area = cv2.contourArea(cnt)
                        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                        if area > 2000:
                            if len(approx) == 4:
                                puckSatArr[color_idx, i, j] += 100
                            elif len(approx) > 5:
                                puckSatArr[color_idx, i, j] += 1
        return puckSatArr

    def decorate(self, puckSatArr):
        h, w = self.Frame.shape[:2]
        height_ratios = [0.25, 0.35, 0.40]
        width_ratios = [0.20, 0.30, 0.30, 0.20]
        height_bounds = [int(sum(height_ratios[:i]) * h) for i in range(4)]
        width_bounds = [int(sum(width_ratios[:i]) * w) for i in range(5)]
        # Display number of detected shapes
        for i in range(3):
            for j in range(4):
                y1, y2 = height_bounds[i], height_bounds[i+1]
                x1, x2 = width_bounds[j], width_bounds[j+1]
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                # Red cell
                red_val = puckSatArr[0, i, j]
                red_bases = red_val // 100
                red_circles = red_val % 100
                # Blue cell
                blue_val = puckSatArr[1, i, j]
                blue_bases = blue_val // 100
                blue_circles = blue_val % 100
                # Draw counts
                cv2.putText(self.Frame, f"{red_circles}", (cx-30, cy), cv2.FONT_ITALIC, 0.8, (0,0,255), 1)
                cv2.putText(self.Frame, f"{red_bases}", (cx-10, cy), cv2.FONT_ITALIC, 0.6, (0,0,255), 2)
                cv2.putText(self.Frame, f"{blue_circles}", (cx+30, cy), cv2.FONT_ITALIC, 0.8, (255,0,0), 1)
                cv2.putText(self.Frame, f"{blue_bases}", (cx+10, cy), cv2.FONT_ITALIC, 0.6, (255,0,0), 2)

        # Draw grid lines
        for y in height_bounds:
            cv2.line(self.Frame, (0, y), (w, y), (255, 255, 255), 2)
        for x in width_bounds:
            cv2.line(self.Frame, (x, 0), (x, h), (255, 255, 255), 2)

    def detect_masked_shapes(self, mask, color=None):
        kernel = np.ones((5,5),np.uint8)
        mask_kernel = cv2.erode(mask, kernel)
        contours, _ = cv2.findContours(mask_kernel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        shapes = []
        color_map = {'red': (0,0,255), 'blue': (255,0,0)}
        draw_color = color_map.get(color, (0,255,0))
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            if area > 2000:
                cv2.drawContours(self.Frame, [cnt], -1, draw_color, 2)
                if len(approx) == 4:
                    shapes.append('quadrilateral')
                elif 8 < len(approx) < 16:
                    shapes.append('circle')
                elif 3 < len(approx) < 5:
                    shapes.append('rectangle')
        return shapes

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
                #print("ok")
                if ratio >= 0.9 and ratio <= 1.1:
                    cv2.drawContours(self.Frame, [cnt], -1, (0,255,255), 3)
                    #cv2.putText(self.Frame, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    pass
                    #cv2.putText(self.Frame, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

def main():
    Cam_obj = CameraVisionModule(0)
    import time
    last_grid_print = 0
    last_calib = 0
    while True:
        Cam_obj.Read_Frame()
        now = time.time()
        # Force calibration every 10 seconds
        if now - last_calib >= 10:
            Cam_obj._cached_mask = {'red': None, 'blue': None}
            Cam_obj.Read_Frame()  # Ensure frame is valid before calibration
            Cam_obj.auto_calibrate()
            last_calib = now
        Cam_obj.Detect_rectangle()
        puck_grid = Cam_obj.Grid_Puck_Count()
        Cam_obj.decorate(puck_grid)
        cv2.imshow("code_test", Cam_obj.Frame)
        if cv2.waitKey(1) == ord('q'):
            Cam_obj.Camera_Cap.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()