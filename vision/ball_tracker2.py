try:
    import cv2
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "opencv-python"])

try:
    import numpy
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy"])
import cv2
import numpy as np
import time
import os
import sys
from collections import deque
from threading import Thread


grid_mm = 320
MIN_MOVEMENT_MM = 0
VELOCITY_FILTER_WINDOW = 1
NUM_AVG_FRAMES = 3

prev_pos_ball = None
prev_pos_laser = None
prev_time = time.time()
use_morpho = True
frame_buffer = []
selected_hsv = None
velocity_history_x = deque(maxlen=VELOCITY_FILTER_WINDOW)
velocity_history_y = deque(maxlen=VELOCITY_FILTER_WINDOW)
camera_port = 0
camera_selection_active = True


def list_available_cameras(max_to_test=5):
    available_ports = []

    log_level = cv2.getLogLevel()
    # Suppress warnings temporarily
    cv2.setLogLevel(0)

    for port in range(max_to_test):
        try:
            cap = cv2.VideoCapture(port, cv2.CAP_DSHOW)
            if cap.isOpened():
                available_ports.append(port)
                cap.release()
            else:
                cap = cv2.VideoCapture(port)
                if cap.isOpened():
                    available_ports.append(port)
                    cap.release()
        except:
            pass
    cv2.setLogLevel(log_level)
    return available_ports


def show_camera_selection():
    """Display camera selection GUI with live preview using OpenCV"""
    global camera_port, camera_selection_active

    cameras = list_available_cameras()
    if not cameras:
        cameras = [0]  

    cv2.namedWindow("Camera Selection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Selection", 800, 600)

    cv2.createTrackbar("Camera Port", "Camera Selection", 0, len(cameras)-1, lambda x: None)

    caps = []
    for port in cameras:
        cap = cv2.VideoCapture(port, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(port)
        caps.append(cap)


    help_text = f"Available cameras: {cameras}\nPress ENTER to confirm selection\nESC to cancel"

    preview_width = 640
    preview_height = 480

    while camera_selection_active:
        selected_idx = cv2.getTrackbarPos("Camera Port", "Camera Selection")
        selected_port = cameras[selected_idx]
        ret, frame = caps[selected_idx].read()
        if not ret:
            frame = np.zeros((preview_height, preview_width, 3), np.uint8)
            cv2.putText(frame, "No signal from camera", (50, preview_height//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        preview = cv2.resize(frame, (preview_width, preview_height))


        display_img = np.zeros((600, 800, 3), np.uint8)

        display_img[0:preview_height, 0:preview_width] = preview

        y = preview_height + 30
        for line in help_text.split('\n'):
            cv2.putText(display_img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y += 30

        cv2.putText(display_img, f"Currently viewing: Camera {selected_port}",
                    (10, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Camera Selection", display_img)

        key = cv2.waitKey(1) & 0xFF
        if key == 13:  
            camera_port = selected_port
            camera_selection_active = False
            break
        elif key == 27: 
            camera_port = None
            camera_selection_active = False
            break

    for cap in caps:
        cap.release()
    cv2.destroyWindow("Camera Selection")

class VideoStream:
    """Threaded video stream to improve FPS"""

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        if not self.stream.isOpened():
            self.stream = cv2.VideoCapture(src)

        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.stream.set(cv2.CAP_PROP_FPS, 60)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()


def setup_hsv_sliders():
    """Initialize the HSV adjustment windows with trackbars"""
    cv2.namedWindow("ball hsv settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ball hsv settings", 400, 300)

    cv2.createTrackbar("ball h low", "ball hsv settings", 12, 179, lambda x: None)
    cv2.createTrackbar("ball s low", "ball hsv settings", 90, 255, lambda x: None)
    cv2.createTrackbar("ball v low", "ball hsv settings", 90, 255, lambda x: None)
    cv2.createTrackbar("ball h high", "ball hsv settings", 40, 179, lambda x: None)
    cv2.createTrackbar("ball s high", "ball hsv settings", 255, 255, lambda x: None)
    cv2.createTrackbar("ball v high", "ball hsv settings", 255, 255, lambda x: None)

    cv2.namedWindow("laser hsv settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("laser hsv settings", 400, 300)

    cv2.createTrackbar("laser h low", "laser hsv settings", 2, 179, lambda x: None)
    cv2.createTrackbar("laser s low", "laser hsv settings", 0, 255, lambda x: None)
    cv2.createTrackbar("laser v low", "laser hsv settings", 16, 255, lambda x: None)
    cv2.createTrackbar("laser h high", "laser hsv settings", 7, 179, lambda x: None)
    cv2.createTrackbar("laser s high", "laser hsv settings", 255, 255, lambda x: None)
    cv2.createTrackbar("laser v high", "laser hsv settings", 255, 255, lambda x: None)

    cv2.namedWindow("processing settings", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("processing settings", 400, 100)
    cv2.createTrackbar("use morpho", "processing settings", 1, 1, lambda x: None)
    cv2.createTrackbar("denoise", "processing settings", 0, 1, lambda x: None)


def get_hsv_limits(target):
    """Get current HSV threshold values from trackbars for specified target (ball/laser)"""
    window = f"{target} hsv settings"
    h_l = cv2.getTrackbarPos(f"{target} h low", window)
    s_l = cv2.getTrackbarPos(f"{target} s low", window)
    v_l = cv2.getTrackbarPos(f"{target} v low", window)
    h_h = cv2.getTrackbarPos(f"{target} h high", window)
    s_h = cv2.getTrackbarPos(f"{target} s high", window)
    v_h = cv2.getTrackbarPos(f"{target} v high", window)

    global use_morpho
    use_morpho = cv2.getTrackbarPos("use morpho", "processing settings") == 1

    # Ensure minimum ranges
    h_h = max(h_h, h_l + 1)
    s_h = max(s_h, s_l + 1)
    v_h = max(v_h, v_l + 1)

    return np.array([h_l, s_l, v_l]), np.array([h_h, s_h, v_h])


def preprocess_frame(frame):
    """Optimized frame preprocessing pipeline"""
    global frame_buffer

    # Frame averaging for stability (reduced buffer size)
    frame_buffer.append(frame)
    if len(frame_buffer) > NUM_AVG_FRAMES:
        frame_buffer.pop(0)

    # Faster averaging using numpy
    if len(frame_buffer) > 1:
        avg_frame = np.mean(frame_buffer, axis=0).astype(np.uint8)
    else:
        avg_frame = frame

    # Conditional denoising (off by default)
    if cv2.getTrackbarPos("denoise", "processing settings") == 1:
        # Using faster GaussianBlur instead of bilateralFilter
        return cv2.GaussianBlur(avg_frame, (5, 5), 0)
    return avg_frame


def pick_color_from_frame(event, x, y, flags, param):
    """Callback to pick HSV values from the frame on mouse click."""
    global selected_hsv
    if event == cv2.EVENT_LBUTTONDOWN:
        frame = param
        if frame is not None and y < frame.shape[0] and x < frame.shape[1]:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            selected_hsv = hsv[y, x]
            print(f"Selected HSV: {selected_hsv}")


def find_object(img, low, high):
    """Optimized object detection pipeline"""
    # Convert to HSV and threshold
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)

    # Conditional morphological operations
    if use_morpho:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if cnts:
        # Find largest contour by area
        biggest = max(cnts, key=cv2.contourArea)
        m = cv2.moments(biggest)
        if m["m00"] > 0:
            # Calculate position in mm coordinates
            px = int(m["m10"] / m["m00"])
            py = int(m["m01"] / m["m00"])
            side = min(img.shape[0], img.shape[1])
            x = (px / side) * grid_mm - (grid_mm / 2)
            y = (grid_mm / 2) - (py / side) * grid_mm
            return (x, y), mask
    return None, mask


def draw_visualization(frame, ball_pos, laser_pos):
    """Optimized drawing of visualization elements"""
    # Draw grid
    step = grid_mm // 9
    for i in range(0, grid_mm + 1, step):
        px = int(i * frame.shape[1] / grid_mm)
        cv2.line(frame, (px, 0), (px, frame.shape[0]), (100, 100, 100), 1)
        cv2.line(frame, (0, px), (frame.shape[1], px), (100, 100, 100), 1)

    mid_x, mid_y = frame.shape[1] // 2, frame.shape[0] // 2
    cv2.line(frame, (mid_x, 0), (mid_x, frame.shape[0]), (0, 255, 0), 2)
    cv2.line(frame, (0, mid_y), (frame.shape[1], mid_y), (0, 255, 0), 2)

    if ball_pos:
        cx = int((ball_pos[0] + grid_mm / 2) * frame.shape[1] / grid_mm)
        cy = int((grid_mm / 2 - ball_pos[1]) * frame.shape[0] / grid_mm)
        cv2.circle(frame, (cx, cy), 10, (0, 255, 255), -1)

    if laser_pos:
        lx = int((laser_pos[0] + grid_mm / 2) * frame.shape[1] / grid_mm)
        ly = int((grid_mm / 2 - laser_pos[1]) * frame.shape[0] / grid_mm)
        cv2.circle(frame, (lx, ly), 5, (0, 0, 255), -1)

    # Status text
    morpho_status = "ON" if use_morpho else "OFF"
    denoise_status = "ON" if cv2.getTrackbarPos("denoise", "processing settings") == 1 else "OFF"
    cv2.putText(frame, f"Morpho: {morpho_status} | Denoise: {denoise_status}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


def get_speed(now, before, now_time, before_time):
    """Calculate filtered velocity with movement threshold"""
    if before is None or now_time == before_time:
        velocity_history_x.append(0)
        velocity_history_y.append(0)
        return (0, 0)

    dt = now_time - before_time
    dx = (now[0] - before[0]) / dt
    dy = (now[1] - before[1]) / dt

    return (dx, dy)


def run():
    """Main processing loop with all optimizations"""
    global prev_pos_ball, prev_pos_laser, prev_time, camera_port

    # Show camera selection GUI
    show_camera_selection()

    if camera_port is None:
        print("Camera selection cancelled. Exiting.")
        return

    # Initialize threaded video stream
    print(f"Initializing camera on port {camera_port}...")
    vs = VideoStream(camera_port).start()
    time.sleep(1.0)  # Allow camera to warm up

    # Setup UI
    setup_hsv_sliders()
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)  # Unbuffered output

    frame_count = 0
    last_hsv_update = 0
    ball_low, ball_high = get_hsv_limits("ball")
    laser_low, laser_high = get_hsv_limits("laser")

    # FPS calculation variables
    fps_counter = 0
    fps = 0
    start_time = time.time()

    while True:
        # Read frame from threaded stream
        frame = vs.read()
        if frame is None:
            break

        # Preprocess frame (cropping happens here)
        processed_frame = preprocess_frame(frame)
        side = min(processed_frame.shape[0], processed_frame.shape[1])
        sx = (processed_frame.shape[1] - side) // 2
        sy = (processed_frame.shape[0] - side) // 2
        processed_frame = processed_frame[sy:sy + side, sx:sx + side]

        # Only update HSV limits every 5 frames to reduce overhead
        if frame_count - last_hsv_update >= 5:
            ball_low, ball_high = get_hsv_limits("ball")
            laser_low, laser_high = get_hsv_limits("laser")
            last_hsv_update = frame_count

        # Detect ball and laser
        ball, ball_mask = find_object(processed_frame, ball_low, ball_high)
        laser, laser_mask = find_object(processed_frame, laser_low, laser_high)

        # Process position and velocity
        now_time = time.time()

        # Ball tracking
        if ball:
            ball_speed = get_speed(ball, prev_pos_ball, now_time, prev_time) if prev_pos_ball else (0, 0)
            dt = now_time - prev_time
            if laser:
                print(f"{ball[0]:.2f},{ball[1]:.2f},{ball_speed[0]:.2f},{ball_speed[1]:.2f},{dt:.9f},{True},{True},{laser[0]},{laser[1]}")
            else:
                print(f"{ball[0]:.2f},{ball[1]:.2f},{ball_speed[0]:.2f},{ball_speed[1]:.2f},{dt:.9f},{True},{False},0.00,0.00")

            prev_pos_ball = ball
        else:
            print(f"0.00,0.00,0.00,0.00,0.00,{False},{False},0.00,0.00")

        prev_time = now_time

        # Calculate FPS (average over 1 second)
        fps_counter += 1
        if (now_time - start_time) > 1.0:
            fps = fps_counter / (now_time - start_time)
            fps_counter = 0
            start_time = now_time

        # Draw visualization (every frame)
        draw_visualization(processed_frame, ball, laser)

        # Display FPS on the processed frame
        cv2.putText(processed_frame, f"FPS: {fps:.1f}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display results (every frame)
        cv2.imshow("tracking", processed_frame)
        cv2.imshow("ball mask", ball_mask)
        cv2.imshow("laser mask", laser_mask)
        cv2.setMouseCallback("tracking", pick_color_from_frame, processed_frame)

        # Exit on ESC
        if cv2.waitKey(1) == 27:
            break

        frame_count += 1

    # Cleanup
    vs.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run()
