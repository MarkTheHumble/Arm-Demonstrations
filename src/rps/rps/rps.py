import threading
import time
from random import randrange 
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

# hand landmarker tracking logic
# https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker

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

moving = False
stage = 0
finishTime = time.time()
interval = 3.0

STARTING_POSITION = {
    "shoulder_pan_joint": 0 * (np.pi / 180), #set to 0 when not sitting at desk # -115 when at desk
    "shoulder_lift_joint": -np.pi / 2,
    "elbow_joint": 0,
    "wrist_1_joint": -np.pi / 2,
    "wrist_2_joint": 0,
    "wrist_3_joint": 0,
}

LOOK_UP = {
    "shoulder_pan_joint": 0 * (np.pi / 180), #set to 0 when not sitting at desk
    "shoulder_lift_joint": -20 * (np.pi / 180),
    "elbow_joint": -70 * (np.pi / 180), # absolute value of this goes higher when adjusting while other 3 go lower. Assumed to be aiming for smaller movements
    "wrist_1_joint": -np.pi / 2,
    "wrist_2_joint": 0,
    "wrist_3_joint": 0,
}

LOOK_DOWN = {
    "shoulder_pan_joint": 0 * (np.pi / 180), #set to 0 when not sitting at desk
    "shoulder_lift_joint": 20 * (np.pi / 180),
    "elbow_joint": -110 * (np.pi / 180),
    "wrist_1_joint": -np.pi / 2,
    "wrist_2_joint": 0,
    "wrist_3_joint": 0,
}

LOOK_DOWNL = {
    "shoulder_pan_joint": 0 * (np.pi / 180), #set to 0 when not sitting at desk
    "shoulder_lift_joint": 20 * (np.pi / 180),
    "elbow_joint": -110 * (np.pi / 180),
    "wrist_1_joint": -np.pi / 2,
    "wrist_2_joint": 0,
    "wrist_3_joint": -120 * (np.pi / 180),
}

LOOK_DOWNR = {
    "shoulder_pan_joint": 0 * (np.pi / 180), #set to 0 when not sitting at desk
    "shoulder_lift_joint": 20 * (np.pi / 180),
    "elbow_joint": -110 * (np.pi / 180),
    "wrist_1_joint": -np.pi / 2,
    "wrist_2_joint": 0,
    "wrist_3_joint": 120 * (np.pi / 180),
}

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

IMAGE_W = 640
IMAGE_H = 480
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


def main():

    global latest_target
    global stage
    global moving
    global finishTime
    global interval

    rclpy.init()

    logger = rclpy.logging.get_logger("Rock_Paper_Scissors")

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

        params.max_velocity_scaling_factor = 1
        params.max_acceleration_scaling_factor = 1

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

            start_time = time.monotonic()

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
                
                #too much time idling
                if stage != 0 and time.time() - finishTime >= interval:
                    stage = 0
                    go_to_joint_pose(ur, arm, logger, STARTING_POSITION, params) 
                    finishTime = time.time() 
                
                if result is not None and result.hand_landmarks:

                    hand = result.hand_landmarks[0]

                    lowest = 0
                    highest = 0

                    for index, item in enumerate(hand):
                        if item.y * 100 < hand[highest].y * 100:
                            highest = index

                        if item.y * 100 > hand[lowest].y * 100:
                            lowest = index

                    # print(f"Lowest index {lowest}, highest index {highest}",flush=True)

                    #initial stage
                    if stage == 0:
                        moving = True 
                        stage = 1

                    #thumbs up for getting the arm's attention
                    if stage == 1 and (lowest == 18 or lowest == 17) and highest == 4 and moving == False:
                        stage = 2
                        moving = True
                        go_to_joint_pose(ur, arm, logger, LOOK_DOWN, params)
                        finishTime = time.time()

                    if stage == 2 and (lowest == 18 or lowest == 17) and highest < 5 and moving == False and time.time() - finishTime >= 0.25:
                        moving = True
                        stage = 3
                        go_to_joint_pose(ur, arm, logger, LOOK_UP, params)
                        go_to_joint_pose(ur, arm, logger, LOOK_DOWN, params)
                        go_to_joint_pose(ur, arm, logger, LOOK_UP, params)
                        go_to_joint_pose(ur, arm, logger, LOOK_DOWN, params)
                        go_to_joint_pose(ur, arm, logger, LOOK_UP, params)

                        match randrange(3):
                            case 0:
                                go_to_joint_pose(ur, arm, logger, LOOK_DOWNL, params)
                            case 1:
                                go_to_joint_pose(ur, arm, logger, LOOK_DOWNR, params)
                            case 2:
                                go_to_joint_pose(ur, arm, logger, LOOK_DOWN, params)

                        finishTime = time.time()


                    moving = False


                draw_landmarks(frame, result)
                cv2.putText(
                    frame,
                    f"FPS: {CAMERA_FPS:.1f}",
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
