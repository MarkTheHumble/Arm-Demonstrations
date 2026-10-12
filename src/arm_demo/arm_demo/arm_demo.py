import threading
import time

import cv2
import depthai as dai
import mediapipe as mp
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import CollisionObject
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Working Area
# botlef: 0.50352; -0.2518; 0.028177
# botrig: 0.49896; 0.2607; 0.028141
# toprig: 0.12029; 0.27638; 0.017567
# toplef: 0.1244; -0.27369; 0.01856
# height: 0.25

WORLD_TOP_LEFT_X = 0.12
WORLD_TOP_LEFT_Y = -0.27
WORLD_TOP_LEFT_Z = 0.10
WORLD_BOTTOM_RIGHT_X = 0.50
WORLD_BOTTOM_RIGHT_Y = 0.27
WORLD_BOTTOM_RIGHT_Z = 0.30

PALM_MIN = 0.4
PALM_MAX = 1.9



LOOK_ONE = {
    "shoulder_pan_joint": np.pi,
    "shoulder_lift_joint": -np.pi / 2,
    "elbow_joint": -np.pi / 4,
    "wrist_1_joint": -np.pi / 2,
    "wrist_2_joint": np.pi / 2,
    "wrist_3_joint": 0,
}

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

IMAGE_W = 640
IMAGE_H = 480

WORLD_CENTER_X = (WORLD_TOP_LEFT_X + WORLD_BOTTOM_RIGHT_X) / 2 + WORLD_TOP_LEFT_X
WORLD_CENTER_Y = (WORLD_TOP_LEFT_Y + WORLD_BOTTOM_RIGHT_Y) / 2 + WORLD_TOP_LEFT_Y
WORLD_CENTER_Z = (WORLD_TOP_LEFT_Z + WORLD_BOTTOM_RIGHT_Z) / 2 + WORLD_TOP_LEFT_Z

MOVE_INTERVAL = 0.00
MIN_MOVE_DISTANCE = 0.01
MIN_ROT_DISTANCE = 0.00001

CAMERA_FPS = 30

mp_hands = mp.tasks.vision.HandLandmarksConnections
mp_drawing = mp.tasks.vision.drawing_utils
mp_drawing_styles = mp.tasks.vision.drawing_styles

MODEL_PATH = "hand_landmarker.task"

options = vision.HandLandmarkerOptions(
    base_options=python.BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=vision.RunningMode.VIDEO,
    num_hands=1,
)

landmarker = vision.HandLandmarker.create_from_options(options)

latest_target = None
target_lock = threading.Lock()


class StopControl(Node):
    def __init__(self, moveit):
        super().__init__("stop_service")

        self.trajectory_execution_manager = moveit.get_trajectory_execution_manager()

        self.stop_event = threading.Event()

        self.create_service(Trigger, "/stop", self.stop_cb)

    def stop_cb(self, request, response):
        self.get_logger().warn("Stopping robot")

        self.stop_event.set()

        self.trajectory_execution_manager.stop_execution(auto_clear=True)

        response.success = True
        response.message = "Stopped"

        return response


class SceneManager(Node):
    def __init__(self, moveit):
        super().__init__("scene_manager")

        self.planning_scene_monitor = moveit.get_planning_scene_monitor()

    def add_box(self, obj_id, frame_id, dims, pos):
        with self.planning_scene_monitor.read_write() as scene:

            obj = CollisionObject()

            obj.header.frame_id = frame_id
            obj.id = obj_id

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dims

            pose = Pose()

            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]

            pose.orientation.w = 1.0

            obj.primitives.append(box)
            obj.primitive_poses.append(pose)

            obj.operation = CollisionObject.ADD

            scene.apply_collision_object(obj)

            scene.current_state.update()

        self.get_logger().info(f"Added object: {obj_id}")

    def clear_scene(self):
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()
        self.get_logger().info("Cleared scene")


def go_to_pose(ur, arm, logger, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0, params=None):

    try:

        arm.set_start_state_to_current_state()

        pose_goal = PoseStamped()

        pose_goal.header.frame_id = "base_link"

        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw

        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")

        result = arm.plan(single_plan_parameters=params) if params else arm.plan()
        if result:
            return ur.execute(result.trajectory, controllers=[])

        return False

    except Exception as e:
        logger.error(f"go_to_pose failed: {e}")
        return False


def go_to_joint_pose(ur, arm, logger, joint_values, params=None):
    try:

        arm.set_start_state_to_current_state()

        robot_model = ur.get_robot_model()

        robot_state = RobotState(robot_model)

        robot_state.joint_positions = joint_values

        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=robot_model.get_joint_model_group("ur_arm"),
        )

        arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        result = arm.plan(single_plan_parameters=params) if params else arm.plan()

        if result:
            return ur.execute(result.trajectory, controllers=[])

        return False

    except Exception as e:

        logger.error(f"go_to_joint_pose failed: {e}")

        return False


def draw_landmarks(frame, detection_result):

    if not detection_result:
        return frame

    for hand_landmarks in detection_result.hand_landmarks:

        mp_drawing.draw_landmarks(
            frame,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style(),
        )

    return frame


def robot_worker(
    ur,
    arm,
    logger,
    params,
    stop_event,
):
    global latest_target
    logger.info("Robot worker started")

    while not stop_event.is_set():

        target = None

        with target_lock:
            if latest_target is not None:
                target = latest_target
                latest_target = None

        if target is None:
            time.sleep(0.01)
            continue

        x, y, z, roll = target
        
        qz = np.clip(np.abs(roll / np.pi), 0, 1.0)

        try:
            logger.info(f"Moving to x={x:.3f}, y={y:.3f}")
            go_to_pose(ur, arm, logger, x, y, z, 1.0, 0.0, qz, 0.0, params=params)

        except Exception as e:
            logger.error(f"Robot worker error: {e}")

    logger.info("Robot worker stopped")


def main():

    global latest_target

    rclpy.init()

    logger = rclpy.logging.get_logger("hand_tracking")

    logger.info("Starting...")

    ur = MoveItPy(node_name="moveit_py")

    arm = ur.get_planning_component("ur_arm")

    logger.info("MoveItPy Ready")

    stop = StopControl(ur)

    scene = SceneManager(ur)

    executor = MultiThreadedExecutor()

    executor.add_node(stop)
    executor.add_node(scene)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)

    spin_thread.start()

    try:
        scene.add_box(
            "table",
            "base_link",
            [2.0, 2.0, 0.01],
            (0.0, 0.0, -0.01),
        )

        params = PlanRequestParameters(ur, "ompl")

        params.max_velocity_scaling_factor = 0.75
        params.max_acceleration_scaling_factor = 0.75

        logger.info("Moving to start pose")

        go_to_joint_pose(ur, arm, logger, LOOK_ONE, params)

        motion_thread = threading.Thread(
            target=robot_worker,
            args=(ur, arm, logger, params, stop.stop_event),
            daemon=True,
        )

        motion_thread.start()

        with dai.Pipeline() as pipeline:

            cam = pipeline.create(dai.node.Camera).build()

            video_queue = cam.requestOutput(
                size=(IMAGE_W, IMAGE_H), fps=CAMERA_FPS
            ).createOutputQueue(maxSize=1, blocking=False)

            pipeline.start()

            logger.info("Camera started")

            start_time = time.monotonic()

            prev_sent_x = WORLD_CENTER_X
            prev_sent_y = WORLD_CENTER_Y
            prev_sent_z = WORLD_CENTER_Z
            prev_sent_roll = 0

            last_move = 0
            now = time.time()

            while pipeline.isRunning() and not stop.stop_event.is_set():

                frame_in = video_queue.tryGet()

                if frame_in is None:
                    time.sleep(0.001)
                    continue

                frame = frame_in.getCvFrame()

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                timestamp_ms = int((time.monotonic() - start_time) * 1000)

                try:
                    mp_image = mp.Image(
                        image_format=mp.ImageFormat.SRGB, data=rgb_frame
                    )
                    result = landmarker.detect_for_video(mp_image, timestamp_ms)
                except Exception as e:
                    logger.error(f"MediaPipe error: {e}")
                    continue

                if result is not None and result.hand_landmarks:

                    hand = result.hand_landmarks[0]

                    # index finger tip landmark
                    wrist = hand[8]

                    palm_indices = [0,1,5,9,13,17]
                    palm_size = sum(
                        np.hypot(
                            hand[b].x - hand[a].x,
                            hand[b].y - hand[a].y
                        ) for a, b in zip(palm_indices, palm_indices[1:] + [palm_indices[0]])
                    )
                    palm_size = (palm_size - PALM_MIN) / (PALM_MAX - PALM_MIN)

                    knuckle_dy = hand[7].y - hand[5].y
                    knuckle_dz = hand[7].z - hand[5].z

                    # Swap X and Y
                    target_x = WORLD_TOP_LEFT_X + (wrist.y * (WORLD_BOTTOM_RIGHT_X - WORLD_TOP_LEFT_X))
                    target_y = WORLD_TOP_LEFT_Y + (wrist.x * (WORLD_BOTTOM_RIGHT_Y - WORLD_TOP_LEFT_Y))
                    target_z = WORLD_TOP_LEFT_Z + (palm_size * (WORLD_BOTTOM_RIGHT_Z - WORLD_TOP_LEFT_Z))
                    target_roll = np.arctan2(knuckle_dz, knuckle_dy)

                    target_x = np.clip(target_x, WORLD_TOP_LEFT_X, WORLD_BOTTOM_RIGHT_X)
                    target_y = np.clip(target_y, WORLD_TOP_LEFT_Y, WORLD_BOTTOM_RIGHT_Y)
                    target_z = np.clip(target_z, WORLD_TOP_LEFT_Z, WORLD_BOTTOM_RIGHT_Z)

                    now = time.time()

                    moved_far_enough = (
                        abs(target_x - prev_sent_x) > MIN_MOVE_DISTANCE
                        or abs(target_y - prev_sent_y) > MIN_MOVE_DISTANCE
                        or abs(target_z - prev_sent_z) > MIN_MOVE_DISTANCE
                        or abs(target_roll - prev_sent_roll) > MIN_ROT_DISTANCE
                    )

                    if now - last_move > MOVE_INTERVAL and moved_far_enough:

                        with target_lock:
                            latest_target = (target_x, target_y, target_z, target_roll)

                        prev_sent_x = target_x
                        prev_sent_y = target_y
                        prev_sent_z = target_z
                        prev_sent_roll = target_roll

                        last_move = now

                draw_landmarks(frame, result)
                fps = 1.0 / max(0.0001, time.time() - now)
                cv2.putText(
                    frame,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("Hand Tracking", frame)

                key = cv2.waitKey(1)
                if key == ord("q"):
                    break
    except KeyboardInterrupt:
        logger.warn("Keyboard interrupt")
    finally:
        logger.info("Shutting down")
        cv2.destroyAllWindows()
        scene.clear_scene()
        stop.stop_event.set()

        executor.shutdown()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
