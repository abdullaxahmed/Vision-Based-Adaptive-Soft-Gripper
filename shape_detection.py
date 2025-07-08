import cv2
import numpy as np
import pyrealsense2 as rs

#  RealSense setup 
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

def detect_one_shape(frame):
    out = frame.copy()

    # Pre‑process: gray → Otsu threshold → dynamic invert → morphology
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if cv2.countNonZero(th) > th.size // 2:
        th = cv2.bitwise_not(th)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    clean  = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=2)
    clean  = cv2.morphologyEx(clean, cv2.MORPH_OPEN,  kernel, iterations=1)

    #  Find all contours 
    cnts, hierarchy = cv2.findContours(clean, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return out
    hier = hierarchy[0]

    # Filter out small contours
    min_area = 2000
    filtered = [(i, c) for i, c in enumerate(cnts) if cv2.contourArea(c) > min_area]
    if not filtered:
        return out

    # Pick the single largest by area
    i_max, cnt = max(filtered, key=lambda ic: cv2.contourArea(ic[1]))

    # Classify that contour
    shape, grasp_pts = None, []
    has_hole = (hier[i_max][2] != -1)

    #  Cylindrical if it has a child contour
    if has_hole:
        shape = "Cylindrical"
        rect = cv2.minAreaRect(cnt)
        box  = cv2.boxPoints(rect).astype(int)
        d01, d12 = np.linalg.norm(box[0]-box[1]), np.linalg.norm(box[1]-box[2])
        if d01 > d12:
            A, B, C, D = box[0], box[1], box[2], box[3]
        else:
            A, B, C, D = box[1], box[2], box[3], box[0]
        m1 = tuple(((A + B) / 2).astype(int))
        m2 = tuple(((C + D) / 2).astype(int))
        grasp_pts = [m1, m2]

    else:
        # approximate polygon
        peri   = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

        # Quadrilateral: square vs flat rectangle
        if len(approx) == 4:
            rect = cv2.minAreaRect(cnt)
            box  = cv2.boxPoints(rect).astype(int)
            d01, d12 = np.linalg.norm(box[0]-box[1]), np.linalg.norm(box[1]-box[2])
            long_e, short_e = max(d01,d12), min(d01,d12)
            ar = long_e / short_e

            if 0.7 < ar < 1.4:
                shape = "Square"
                x,y,w,h = cv2.boundingRect(approx)
                grasp_pts = [
                    (x,     int(y + h/2)),
                    (x + w, int(y + h/2))
                ]
            else:
                shape = "Rectangle"
                # midpoints of **short edges**
                if d01 < d12:
                    pairs = [(box[0],box[1]), (box[2],box[3])]
                else:
                    pairs = [(box[1],box[2]), (box[3],box[0])]
                for P, Q in pairs:
                    grasp_pts.append(tuple(((P+Q)/2).astype(int)))

        else:
            #  >4 sides: circle test
            circ = 4 * np.pi * cv2.contourArea(cnt) / (peri * peri)
            if circ > 0.7:
                shape = "Circle"
                (cx, cy), r = cv2.minEnclosingCircle(cnt)
                cx, cy, r = int(cx), int(cy), int(r)
                for angle in (90, 210, 330):
                    θ = np.deg2rad(angle)
                    gx = int(cx + r * np.cos(θ))
                    gy = int(cy - r * np.sin(θ))
                    grasp_pts.append((gx, gy))


    if shape:
        cv2.drawContours(out, [cnt], -1, (0,255,0), 2)
        for gp in grasp_pts:
            cv2.circle(out, gp, 6, (0,0,255), -1)
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            lx = int(M["m10"]/M["m00"])
            ly = int(M["m01"]/M["m00"]) - 10
        else:
            lx, ly = grasp_pts[0]
        cv2.putText(out, shape, (lx, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

    return out

def main():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            vis   = detect_one_shape(frame)

            cv2.imshow("Single‑Shape Grasp‑Points", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
