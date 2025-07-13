import cv2
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
import serial
import time
import sys


SERIAL_PORT = "COM8"            

BAUD = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    time.sleep(2)              
except serial.SerialException as e:
    sys.exit(f"[ERROR] Can't open serial port {SERIAL_PORT}: {e}")

shape_to_cmd = {"Circle": "S", "Cylindrical": "S",
                "Rectangle": "R", "Square": "Q"}
last_cmd = None               

# Parameters
W, H           = 640, 480
PEAK_BIN_COUNT = 60
PEAK_TOL_M     = 0.009
MAX_DEPTH_M    = 0.28
ERODE_PX       = 0

POINT_SIZE = 1.0
GRID_HALF  = 1.0
GRID_STEP  = 0.1



def make_grid(half, step):
    pts, lines, idx = [], [], 0
    rng = np.arange(-half, half + 1e-6, step)
    for x in rng:
        pts += [[x, -half, 0], [x, half, 0]]
        lines += [[idx, idx + 1]]
        idx += 2
    for y in rng:
        pts += [[-half, y, 0], [half, y, 0]]
        lines += [[idx, idx + 1]]
        idx += 2
    grid = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(pts),
        lines=o3d.utility.Vector2iVector(lines)
    )
    grid.paint_uniform_color([0.6, 0.6, 0.6])
    return grid

# Contour detector 
def detect_and_draw(frame):
    
    out  = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
    if cv2.countNonZero(th) > th.size // 2:
        th = cv2.bitwise_not(th)

    k7     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    clean  = cv2.morphologyEx(th, cv2.MORPH_CLOSE, k7, 2)
    clean  = cv2.morphologyEx(clean, cv2.MORPH_OPEN,  k7, 1)

    cnts, _ = cv2.findContours(clean, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return out, [], None

    cnt   = max(cnts, key=cv2.contourArea)
    peri  = cv2.arcLength(cnt, True)
    approx= cv2.approxPolyDP(cnt, 0.04 * peri, True)

    raw_edge_pts, extra_dots, shape = [], [], None

    # SQUARE / RECTANGLE 
    if len(approx) == 4:
        rect = cv2.minAreaRect(cnt)
        box  = cv2.boxPoints(rect).astype(int)
        d01, d12 = np.linalg.norm(box[0]-box[1]), np.linalg.norm(box[1]-box[2])
        aspect   = max(d01, d12) / min(d01, d12)
        shape    = "Square" if 0.7 < aspect < 1.4 else "Rectangle"

        x, y, w, h = cv2.boundingRect(approx)
        raw_edge_pts = [(x,          y + h // 2),
                        (x + w,      y + h // 2)]

        if shape == "Square":
            extra_dots = [
                (x + w // 2, y),
                (x,          y + h // 2),
                (x + w,      y + h // 2)
            ]
        else:                    
            edges  = [(box[i], box[(i+1) % 4]) for i in range(4)]
            edges  = sorted(edges, key=lambda e: -np.linalg.norm(e[0]-e[1]))
            long1, long2 = edges[:2]

            def avg_y(edge): return (edge[0][1] + edge[1][1]) / 2
            top_edge = long1 if avg_y(long1) < avg_y(long2) else long2
            bot_edge = long2 if top_edge is long1 else long1

            v      = top_edge[1] - top_edge[0]
            p25    = tuple((top_edge[0] + 0.25 * v).astype(int))
            p75    = tuple((top_edge[0] + 0.75 * v).astype(int))
            botmid = tuple(((bot_edge[0] + bot_edge[1]) / 2).astype(int))
            extra_dots = [p25, p75, botmid]

    # CIRCLE / CYLINDER 
    else:
        circ = 4 * np.pi * cv2.contourArea(cnt) / (peri * peri)
        if circ > 0.7:                     
            shape = "Circle"
            (cx, cy), r = cv2.minEnclosingCircle(cnt)
            cx, cy, r = int(cx), int(cy), int(r)
            extra_dots = [
                (cx,              cy - r),
                (int(cx - 0.87*r), int(cy + 0.50*r)),
                (int(cx + 0.87*r), int(cy + 0.50*r))
            ]
            raw_edge_pts = extra_dots[:2]
        else:                              
            shape = "Cylindrical"
            rect = cv2.minAreaRect(cnt)
            box  = cv2.boxPoints(rect).astype(int)
            d01, d12 = np.linalg.norm(box[0]-box[1]), np.linalg.norm(box[1]-box[2])
            A,B,C,D = (box[0],box[1],box[2],box[3]) if d01>d12 else (box[1],box[2],box[3],box[0])
            raw_edge_pts = [tuple(((A+B)/2).astype(int)),
                             tuple(((C+D)/2).astype(int))]
            top_center   = tuple(((A+C)/2).astype(int))
            extra_dots   = raw_edge_pts + [top_center]

   
    if shape:
        cv2.drawContours(out, [cnt], -1, (0, 255, 0), 2)
        for p in extra_dots:
            cv2.circle(out, p, 6, (0, 0, 255), -1)
        tx, ty = extra_dots[0] if extra_dots else (0, 0)
        cv2.putText(out, shape, (tx - 10, ty - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    return out, raw_edge_pts, shape

# RealSense setup
pipe = rs.pipeline()
cfg  = rs.config()
cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
prof = pipe.start(cfg)

d_intr      = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
c_intr      = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()

# Open3D visualiser
vis = o3d.visualization.Visualizer()
vis.create_window("3-D Measure", width=W, height=H)
opt = vis.get_render_option(); opt.background_color = [1, 1, 1]; opt.point_size = POINT_SIZE

grid = make_grid(GRID_HALF, GRID_STEP); vis.add_geometry(grid)
pintr = o3d.camera.PinholeCameraIntrinsic(W, H, c_intr.fx, c_intr.fy, c_intr.ppx, c_intr.ppy)
vis.add_geometry(o3d.geometry.LineSet.create_camera_visualization(
    pintr, np.linalg.inv(np.eye(4)), 0.2))

pcd  = o3d.geometry.PointCloud(); vis.add_geometry(pcd)
s0   = o3d.geometry.TriangleMesh.create_sphere(0.005); s0.paint_uniform_color([1, 0, 0])
s1   = o3d.geometry.TriangleMesh.create_sphere(0.005); s1.paint_uniform_color([1, 0, 0])
lset = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector([[0, 0, 0], [0, 0, 0]]),
                            lines=o3d.utility.Vector2iVector([[0, 1]]))
vis.add_geometry(s0); vis.add_geometry(s1); vis.add_geometry(lset)

try:
    while True:
        fs = pipe.wait_for_frames()
        d  = fs.get_depth_frame()
        c  = fs.get_color_frame()
        if not d or not c:
            continue

        img = np.asarray(c.get_data())
        rgb_view, _, shape = detect_and_draw(img)   

        
        gmask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, bin_ = cv2.threshold(gmask, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
        if cv2.countNonZero(bin_) > bin_.size // 2:
            bin_ = cv2.bitwise_not(bin_)
        if ERODE_PX:
            bin_ = cv2.erode(bin_, np.ones((ERODE_PX, ERODE_PX), np.uint8))
        cnt = (max(cv2.findContours(bin_, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[0],
                   key=cv2.contourArea) if np.count_nonzero(bin_) else None)
        if cnt is not None:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy     = x + w // 2, y + h // 2
            d_np = np.asarray(d.get_data(), dtype=np.float32) * depth_scale
            patch = d_np[y:y + h, x:x + w]
            vals  = patch[(patch > 0) & (patch < MAX_DEPTH_M)]
            if vals.size:
                hist, bins = np.histogram(vals, PEAK_BIN_COUNT, (0, MAX_DEPTH_M))
                peak = 0.5 * (bins[hist.argmax()] + bins[hist.argmax() + 1])
                obj  = (patch > peak - PEAK_TOL_M) & (patch < peak + PEAK_TOL_M)
                row  = int(obj.shape[0] * 0.35)
                xs   = np.where(obj[row])[0]
                if xs.size >= 2:
                    p0 = (x + int(xs[0]),     y + row)
                    p1 = (x + int(xs[-1]),    y + row)
                    z0 = d.get_distance(*p0)
                    z1 = d.get_distance(*p1)
                    P0 = np.array(rs.rs2_deproject_pixel_to_point(d_intr, list(p0), z0))
                    P1 = np.array(rs.rs2_deproject_pixel_to_point(d_intr, list(p1), z1))
                    dist_cm = np.linalg.norm(P1 - P0) * 100.0
                    cv2.putText(rgb_view, f"{dist_cm:.1f} cm",
                                (cx - 40, cy + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                
                    pc = rs.pointcloud(); pc.map_to(c)
                    v  = np.asarray(pc.calculate(d).get_vertices()) \
                           .view(np.float32).reshape(-1, 3)
                    col = np.asarray(c.get_data()).reshape(-1, 3) / 255.0
                    good = v[:, 2] < MAX_DEPTH_M
                    pcd.points = o3d.utility.Vector3dVector(v[good])
                    pcd.colors = o3d.utility.Vector3dVector(col[good])
                    s0.translate(P0, relative=False); s1.translate(P1, relative=False)
                    lset.points = o3d.utility.Vector3dVector([P0, P1])
                    vis.update_geometry(pcd); vis.update_geometry(s0)
                    vis.update_geometry(s1);  vis.update_geometry(lset)

        vis.poll_events(); vis.update_renderer()


        if shape in shape_to_cmd:
            cmd = shape_to_cmd[shape]
            if cmd != last_cmd:
                try:
                    ser.write((cmd + "\n").encode())
                    last_cmd = cmd
                    print(f"[Serial] Sent '{cmd}'  ({shape})")
                except serial.SerialException:
                    print("[WARN] Serial write failed")

    
        cv2.imshow("RGB View", rgb_view)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
    vis.destroy_window()
    ser.close()
