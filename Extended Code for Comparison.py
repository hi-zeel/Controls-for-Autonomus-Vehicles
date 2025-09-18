import carla
import numpy as np
import cv2
import math
import time
import matplotlib.pyplot as plt

# ==================================================
# STEP 1: SETUP CARLA CLIENT, VEHICLE & CAMERA
# ==================================================
def setup_carla():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    vehicle_bp = blueprint_library.filter('model3')[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '360')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    return vehicle, camera, vehicle_bp.id


# ==================================================
# STEP 2: PID CONTROLLER CLASS
# ==================================================
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0
        self.prev_time = None

    def control(self, error):
        curr_time = time.time()
        dt = curr_time - self.prev_time if self.prev_time else 0.05
        self.prev_time = curr_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


# ==================================================
# STEP 3: LANE DETECTION FUNCTION
# ==================================================
def detect_lane(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    frame = array.reshape((image.height, image.width, 4))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    h, w = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (int(w*0.1), h),
        (int(w*0.9), h),
        (int(w*0.6), int(h*0.6)),
        (int(w*0.4), int(h*0.6))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(cropped, 1, np.pi/180, 50, minLineLength=50, maxLineGap=150)
    left_lines, right_lines = [], []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-5)
            if slope < -0.5:
                left_lines.append(line[0])
            elif slope > 0.5:
                right_lines.append(line[0])

    def average_line(lines):
        if len(lines) == 0:
            return None
        x_coords, y_coords = [], []
        for x1, y1, x2, y2 in lines:
            x_coords += [x1, x2]
            y_coords += [y1, y2]
        poly = np.polyfit(y_coords, x_coords, 1)
        slope, intercept = poly[0], poly[1]
        y1, y2 = h, int(h*0.6)
        x1, x2 = int(slope*y1 + intercept), int(slope*y2 + intercept)
        return (x1, y1, x2, y2)

    left_avg = average_line(left_lines)
    right_avg = average_line(right_lines)

    lane_center = None
    if left_avg and right_avg:
        lx1, _, rx1, _ = left_avg[0], left_avg[1], right_avg[0], right_avg[1]
        lane_center = ((lx1 + rx1) // 2, h)

    return frame, lane_center, w, h


# ==================================================
# STEP 4: RUN EXPERIMENT (WITH OR WITHOUT PID)
# ==================================================
def run_experiment(use_pid=True, duration=15):
    vehicle, camera, vehicle_name = setup_carla()
    pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.2) if use_pid else None
    frame_holder = {"frame": None}
    start_time = time.time()

    offsets, angles, times = [], [], []

    def process_image(image):
        frame, lane_center, w, h = detect_lane(image)
        frame_holder["frame"] = frame

        if lane_center:
            car_center_x = w // 2
            dx = lane_center[0] - car_center_x
            dy = h - lane_center[1]
            steering_angle = math.degrees(math.atan2(dx, dy))
            offset = dx * (3.7 / w)

            offsets.append(offset)
            angles.append(steering_angle)
            times.append(time.time() - start_time)

            if use_pid and pid:
                steer_cmd = pid.control(offset)
                steer_cmd = max(-1.0, min(1.0, steer_cmd / 5))
                control = carla.VehicleControl(throttle=0.4, steer=steer_cmd)
                vehicle.apply_control(control)

    camera.listen(lambda image: process_image(image))

    try:
        if not use_pid:
            vehicle.set_autopilot(True)  # autopilot for no PID
        else:
            vehicle.set_autopilot(False)  # PID takes control
        print(f"Running experiment - PID={use_pid}. Press 'q' to stop.")
        while time.time() - start_time < duration:
            if frame_holder["frame"] is not None:
                cv2.imshow(f"Lane Detection - PID={use_pid}", frame_holder["frame"])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.stop()
        vehicle.destroy()
        cv2.destroyAllWindows()

    return times, offsets, angles


# ==================================================
# STEP 5: COMPARE AND PLOT RESULTS
# ==================================================
def main():
    # Case 1: Without PID
    times_no_pid, offsets_no_pid, angles_no_pid = run_experiment(use_pid=False, duration=15)

    # Case 2: With PID
    times_pid, offsets_pid, angles_pid = run_experiment(use_pid=True, duration=15)

    # Plot results
    plt.figure(figsize=(12,5))

    # Lane Offset Plot
    plt.subplot(1,2,1)
    plt.plot(times_no_pid, offsets_no_pid, label="Without PID")
    plt.plot(times_pid, offsets_pid, label="With PID")
    plt.xlabel("Time (s)")
    plt.ylabel("Lane Offset (m)")
    plt.title("Lane Offset Comparison")
    plt.legend()
    plt.grid(True)

    # Steering Angle Plot
    plt.subplot(1,2,2)
    plt.plot(times_no_pid, angles_no_pid, label="Without PID")
    plt.plot(times_pid, angles_pid, label="With PID")
    plt.xlabel("Time (s)")
    plt.ylabel("Steering Angle (deg)")
    plt.title("Steering Angle Comparison")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
