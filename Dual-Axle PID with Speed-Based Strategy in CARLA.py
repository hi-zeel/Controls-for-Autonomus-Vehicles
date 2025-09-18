import carla
import numpy as np
import cv2
import math
import time
import matplotlib.pyplot as plt

# ==================================================
# STEP 1: SETUP CARLA CLIENT, VEHICLE & CAMERA
# ==================================================
def setup_carla(spawn_point=None):
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    vehicle_bp = blueprint_library.filter('model3')[0]
    if spawn_point is None:
        spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '360')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    return client, world, vehicle, camera, vehicle_bp.id


# ==================================================
# STEP 2: PID CONTROLLER
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
# STEP 3: RECORD AUTOPILOT WAYPOINTS
# ==================================================
def record_autopilot(spawn_point, duration=15):
    client, world, vehicle, camera, _ = setup_carla(spawn_point)
    waypoints = []
    start_time = time.time()

    vehicle.set_autopilot(True)
    camera.listen(lambda image: None)

    try:
        while time.time() - start_time < duration:
            loc = vehicle.get_transform().location
            waypoints.append((loc.x, loc.y))
            world.tick()
            time.sleep(0.05)
    finally:
        camera.stop()
        vehicle.destroy()

    return waypoints


# ==================================================
# STEP 4: DUAL-AXLE PID FOLLOW WAYPOINTS + HUD
# ==================================================
def run_dual_axle_pid(spawn_point, waypoints, duration=15):
    client, world, vehicle, camera, vehicle_name = setup_carla(spawn_point)
    pid_front = PIDController(Kp=0.5, Ki=0.01, Kd=0.2)
    start_time = time.time()
    frame_holder = {"frame": None}
    idx = 0

    offsets_heading, offsets_lane, angles, times = [], [], [], []

    # Camera callback (store latest frame only)
    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        frame = array.reshape((image.height, image.width, 4))
        frame_holder["frame"] = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    camera.listen(lambda image: camera_callback(image))

    try:
        vehicle.set_autopilot(False)
        print("Running Dual-Axle PID with HUD...")

        while time.time() - start_time < duration and idx < len(waypoints):
            if frame_holder["frame"] is None:
                world.tick()
                continue  # wait until first frame is ready

            loc = vehicle.get_transform().location
            rot = vehicle.get_transform().rotation
            target_x, target_y = waypoints[idx]

            dx = target_x - loc.x
            dy = target_y - loc.y
            desired_heading = math.degrees(math.atan2(dy, dx))
            current_heading = rot.yaw
            heading_error = desired_heading - current_heading

            steer_cmd_f = pid_front.control(heading_error)
            steer_cmd_f = max(-1.0, min(1.0, steer_cmd_f / 45.0))

            velocity = vehicle.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            k = -0.5 if speed < 8 else 0.3
            steer_cmd_r = k * steer_cmd_f

            combined_steer = (steer_cmd_f + steer_cmd_r) / 2
            combined_steer = max(-1.0, min(1.0, combined_steer))

            control = carla.VehicleControl(throttle=0.4, steer=combined_steer)
            vehicle.apply_control(control)

            # Lane offset from image center
            h, w, _ = frame_holder["frame"].shape
            lane_offset = dx * (3.7 / w)

            # HUD overlay
            hud_frame = frame_holder["frame"].copy()
            cv2.putText(hud_frame, f"Vehicle: {vehicle_name}", (20,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(hud_frame, f"Speed: {3.6*speed:.1f} km/h", (20,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(hud_frame, f"Steer Cmd: {combined_steer:.2f}", (20,90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(hud_frame, f"Heading Error: {heading_error:.2f} deg", (20,120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(hud_frame, f"Lane Offset: {lane_offset:.2f} m", (20,150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            cv2.imshow("Dual-Axle PID HUD", hud_frame)

            offsets_heading.append(heading_error)
            offsets_lane.append(lane_offset)
            angles.append(current_heading)
            times.append(time.time() - start_time)

            idx += 1
            world.tick()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.05)

    finally:
        camera.stop()
        vehicle.destroy()
        cv2.destroyAllWindows()

    return times, offsets_lane, offsets_heading, angles


# ==================================================
# STEP 5: MAIN
# ==================================================
def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    spawn_point = world.get_map().get_spawn_points()[0]

    print("Recording Autopilot Path...")
    waypoints = record_autopilot(spawn_point, duration=15)

    print("Running Dual-Axle PID with HUD...")
    times, offsets_lane, offsets_heading, angles = run_dual_axle_pid(spawn_point, waypoints, duration=15)

    plt.figure(figsize=(12,7))

    plt.subplot(1,3,1)
    plt.plot(times, offsets_lane, label="Lane Offset")
    plt.xlabel("Time (s)")
    plt.ylabel("Offset (m)")
    plt.title("Lane Offset vs Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(1,3,2)
    plt.plot(times, offsets_heading, label="Heading Error")
    plt.xlabel("Time (s)")
    plt.ylabel("Heading Error (deg)")
    plt.title("Heading Error vs Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(1,3,3)
    plt.plot(times, angles, label="Vehicle Heading")
    plt.xlabel("Time (s)")
    plt.ylabel("Heading (deg)")
    plt.title("Vehicle Heading vs Time")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()