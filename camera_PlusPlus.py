import cv2
import numpy as np
import time
import sys
import motors

class CameraVisionModule:
    def __init__(self,we = 0, OuterCam=0, InnerCam=1):
        # Outer camera for puck detection
        self.OuterCam_Cap = cv2.VideoCapture(OuterCam)
        _, self.OuterFrame = self.OuterCam_Cap.read()
        self.OuterCam_Cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.OuterCam_Cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Inner camera for puck classification
        self.InnerCam_Cap = cv2.VideoCapture(InnerCam)
        _, self.InnerFrame = self.InnerCam_Cap.read()
        self.InnerCam_Cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.InnerCam_Cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.InnerCam_Status = 'n'  # 'r', 'b', 'n'

        self.hsv_thresholds = {
            'red': [(0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)],
            'blue': [(100, 120, 70), (140, 255, 255)]
        }
        self.height_ratios = [0.25, 0.35, 0.40]
        self.width_ratios = [0.20, 0.20, 0.20, 0.20, 0.20]
        self.last_calib = 0
        self.last_decision = 0
        self.Last_move = 0
        self.puckSatArr = np.zeros((2, 3, 4), dtype=int)  # [color, row, col]

        # Robot motion control parameters
        self.speed = 50
        self.Collect = True
        self.state = 'm'  # 'l', 'r', 'm', 'n'
        self.NoPuckCount = 0
        self.we = we #  0 -> 'red', 1 -> 'blue'

    def decision(self):  # state -> l, r, m, n
        freq = (self.puckSatArr[0]//100) + (self.puckSatArr[1]//100)
        L = freq[0, 0]*6 + freq[1, 0]*7 + freq[2, 0]*8 + freq[1, 1]*9 + freq[2, 1]*10
        M = freq[0, 1]*6 + freq[0, 2]*8 + freq[0, 3]*6 + freq[1, 2]*10 + freq[2, 2]*10
        R = freq[0, 4]*6 + freq[1, 4]*7 + freq[2, 4]*8 + freq[1, 3]*9 + freq[2, 3]*10
        if L > M and L > R:
            self.state = 'l'
        elif R > M and R > L:
            self.state = 'r'
        elif M == 0 and L == 0 and R == 0:
            self.state = 'n'
        elif M >= L and M >= R:
            self.state = 'm'
        else:
            return False
        return True

    def Move(self, frequency = 3):
        now = time.time()
        if now - self.last_calib < frequency:
            return False
        motors.init()
        motors.set_speed(self.speed)
        if self.Collect: 
            if self.state == 'l':
                motors.left()
            elif self.state == 'r':
                motors.right()
            elif self.state == 'm':
                motors.forward()
            elif self.state == 'n':
                self.NoPuckCount += 1
                if self.NoPuckCount > 3:
                    self.Collect = False
                    self.goBack()
            return True
        else: 
            print("Not in collection mode")

    def goBack(self):
        if not self.Collect:
            base_found = self.check_for_base()
            
            if base_found:
                motors.forward()
                time.sleep(2)
            else:
                for _ in range(4):
                    motors.right()
                    time.sleep(0.5)
                    motors.stop()
                    time.sleep(0.3)
                    
                    if self.read_frames():
                        self.grid_puck_count()
                        if self.check_for_base():
                            motors.forward()
                            time.sleep(2)
                            break
            
            motors.stop()
    
    def check_for_base(self):
        for x in range(len(self.height_ratios)):
            for y in range(len(self.width_ratios)):
                if self.puckSatArr[self.we, x, y] >= 100 and y == 3:
                    return True
        return False

    def classify(self):
        if self.inner_cam():
            if self.InnerCam_Status == 'r':
                motors.ServoLeft()
            elif self.InnerCam_Status == 'b':
                motors.ServoRight()
            else:
                motors.ServoCentre()
            return True
        return False

    def read_frames(self):
        O, self.OuterFrame = self.OuterCam_Cap.read()
        I, self.InnerFrame = self.InnerCam_Cap.read()
        if not O or not I:
            return False
        return True

    def auto_calibrate(self, frequency=10): # hsv_thresholds
        now = time.time()
        if now - self.last_calib < frequency:
            return False
        
        self.last_calib = now
        hsv_img = cv2.cvtColor(self.OuterFrame, cv2.COLOR_BGR2HSV)
        pixels = hsv_img.reshape(-1, 3)
        pixels = np.float32(pixels)
        K = 4
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
        _, _, centers = cv2.kmeans(pixels, K, None, criteria, 10, cv2.KMEANS_PP_CENTERS)
        
        target_hsv = {'red': np.array([5, 150, 150]), 'blue': np.array([110, 150, 150])}
        
        for color in ['red', 'blue']:
            distances = [np.linalg.norm(center - target_hsv[color]) for center in centers]
            best_idx = np.argmin(distances)
            h, s, v = centers[best_idx]
            
            if color == 'red':
                h_tolerance = 25
                s_tolerance = 70
                v_tolerance = 70
                
                lower1_h = max(0, int(h) - h_tolerance)
                upper1_h = min(10, int(h) + h_tolerance)
                lower2_h = max(170, 180 - h_tolerance)
                upper2_h = min(180, 180 + h_tolerance)
                
                if int(h) > 170:
                    lower2_h = max(170, int(h) - h_tolerance)
                    upper2_h = min(180, int(h) + h_tolerance)
                
                self.hsv_thresholds['red'] = [
                    (lower1_h, max(60, int(s) - s_tolerance), max(60, int(v) - v_tolerance)),
                    (upper1_h, min(255, int(s) + s_tolerance), min(255, int(v) + v_tolerance)),
                    (lower2_h, max(60, int(s) - s_tolerance), max(60, int(v) - v_tolerance)),
                    (upper2_h, min(255, int(s) + s_tolerance), min(255, int(v) + v_tolerance))
                ]
            else:  # blue
                h_tolerance = 60
                s_tolerance = 100
                v_tolerance = 100
                
                self.hsv_thresholds['blue'] = [
                    (max(100, int(h) - h_tolerance), max(60, int(s) - s_tolerance), max(60, int(v) - v_tolerance)),
                    (min(140, int(h) + h_tolerance), min(255, int(s) + s_tolerance), min(255, int(v) + v_tolerance))
                ]
        
        return True

    def grid_puck_count(self): # puckSatArr
        h, w = self.OuterFrame.shape[:2]
        height_bounds = [int(sum(self.height_ratios[:i]) * h) for i in range(4)]
        width_bounds = [int(sum(self.width_ratios[:i]) * w) for i in range(5)]
        self.puckSatArr = np.zeros((2, 3, 4), dtype=int)
        kernel = np.ones((5,5),np.uint8)
        for color_idx, color in enumerate(['red', 'blue']):
            for i in range(3):
                for j in range(4):
                    y1, y2 = height_bounds[i], height_bounds[i+1]
                    x1, x2 = width_bounds[j], width_bounds[j+1]
                    roi_hsv = cv2.cvtColor(self.OuterFrame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
                    mask = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)

                    if color == 'red':
                        t = self.hsv_thresholds['red']
                        mask |= cv2.inRange(roi_hsv, np.array(t[0]), np.array(t[1]))
                        mask |= cv2.inRange(roi_hsv, np.array(t[2]), np.array(t[3]))
                    else:  # blue
                        t = self.hsv_thresholds['blue']
                        mask |= cv2.inRange(roi_hsv, np.array(t[0]), np.array(t[1]))
                    mask = cv2.erode(mask, kernel)

                    # cv2.imshow(f"{color}_mask_r{i}_c{j}", mask)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                    for cnt in contours:
                        area = cv2.contourArea(cnt)
                        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                        if area > 1000:
                            if len(approx) == 4:
                                self.puckSatArr[color_idx, i, j] += 100
                            elif len(approx) > 5:
                                self.puckSatArr[color_idx, i, j] += 1
        return True

    def inner_cam(self): # InnerCam_Status -> r, b, n
        if self.InnerFrame is None:
            return False
        
        hsv_inner = cv2.cvtColor(self.InnerFrame, cv2.COLOR_BGR2HSV)
        
        # Red
        red_mask1 = cv2.inRange(hsv_inner, np.array([0, 120, 70]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv_inner, np.array([170, 120, 70]), np.array([180, 255, 255]))
        red_mask = red_mask1 | red_mask2
        red_area = cv2.countNonZero(red_mask)
        
        # Blue
        blue_mask = cv2.inRange(hsv_inner, np.array([100, 120, 70]), np.array([140, 255, 255]))
        blue_area = cv2.countNonZero(blue_mask)
        
        thres = 500
        
        if red_area > thres:
            self.InnerCam_Status = 'r'
        elif blue_area > thres:
            self.InnerCam_Status = 'b'
        else:
            self.InnerCam_Status = 'n'
        return True

    def decorate(self):
        # Outer camera decoration
        h, w = self.OuterFrame.shape[:2]
        height_bounds = [int(sum(self.height_ratios[:i]) * h) for i in range(4)]
        width_bounds = [int(sum(self.width_ratios[:i]) * w) for i in range(5)]
        # Display detected shapes
        for i in range(3):
            for j in range(4):
                y1, y2 = height_bounds[i], height_bounds[i+1]
                x1, x2 = width_bounds[j], width_bounds[j+1]
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                # Red cell
                red_val = self.puckSatArr[0, i, j]
                red_bases = red_val // 100
                red_circles = red_val % 100
                # Blue cell
                blue_val = self.puckSatArr[1, i, j]
                blue_bases = blue_val // 100
                blue_circles = blue_val % 100
                # Draw counts
                cv2.putText(self.OuterFrame, f"{red_circles}", (cx-30, cy), cv2.FONT_ITALIC, 0.8, (0,0,255), 1)
                cv2.putText(self.OuterFrame, f"{red_bases}", (cx-10, cy), cv2.FONT_ITALIC, 0.6, (0,0,255), 2)
                cv2.putText(self.OuterFrame, f"{blue_circles}", (cx+30, cy), cv2.FONT_ITALIC, 0.8, (255,0,0), 1)
                cv2.putText(self.OuterFrame, f"{blue_bases}", (cx+10, cy), cv2.FONT_ITALIC, 0.6, (255,0,0), 2)
        # Draw grid lines
        for y in height_bounds:
            cv2.line(self.OuterFrame, (0, y), (w, y), (255, 255, 255), 2)
        for x in width_bounds:
            cv2.line(self.OuterFrame, (x, 0), (x, h), (255, 255, 255), 2)

        # Inner camera decoration
        if self.InnerFrame is not None:
            if self.InnerCam_Status == 'r':
                status_color = (0, 0, 255)    # Red
                status_text = "RED"
            elif self.InnerCam_Status == 'b':
                status_color = (255, 0, 0)    # Blue  
                status_text = "BLUE"
            else:
                status_color = (255, 255, 255)  # White
                status_text = "NONE"
            
            cv2.putText(self.InnerFrame, status_text, (280, 250), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, status_color, 3)

def main():

    we = sys.argv[1]
    Cam_obj = CameraVisionModule(we, 0, 1)
    last_decision = 0
    while True:
        if not Cam_obj.read_frames():
            continue
        
        now = time.time()
        
        Cam_obj.auto_calibrate()  # frequency=10
        Cam_obj.grid_puck_count()
        
        if now - last_decision >= 5:  # frequency=5
            Cam_obj.decision()
            last_decision = now
            print(f"Decision: {Cam_obj.state}, Inner Status: {Cam_obj.InnerCam_Status}")
        
        Cam_obj.Move()
        
        Cam_obj.classify()  # inner_cam is called within classify
        # Cam_obj.decorate()

        # Display both cameras
        cv2.imshow("Inner Camera", Cam_obj.InnerFrame)
        cv2.imshow("Outer Camera", Cam_obj.OuterFrame)
        
        if cv2.waitKey(1) == ord('q'):
            Cam_obj.OuterCam_Cap.release()
            Cam_obj.InnerCam_Cap.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()