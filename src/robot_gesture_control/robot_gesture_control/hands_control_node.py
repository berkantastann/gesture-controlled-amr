import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


LINEAR_SPEED    = 0.3    # m/s
ANGULAR_SPEED   = 0.8    # rad/s
PINCH_THRESHOLD = 0.06   # normalized thumb-index distance
LIFT_EFFORT     = 100.0  # N — lift up/down effort
LIFT_ZONE_X     = 0.82   # normalized x threshold for lift strip

MODEL_PATH = get_package_share_directory('robot_gesture_control') + '/models/hand_landmarker.task'

HAND_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (5,9),(9,10),(10,11),(11,12),
    (9,13),(13,14),(14,15),(15,16),
    (13,17),(17,18),(18,19),(19,20),
    (0,17),
]


class HandsControlNode(Node):
    def __init__(self):
        super().__init__('hands_control_node')
        self.get_logger().info('Hands Control Node started.')

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10
        )
        self.lift_pub = self.create_publisher(
            Float64MultiArray, '/lift_controller/commands', 10
        )

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open webcam!')
            raise RuntimeError('Webcam not found')

        base_options = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
        options = mp_vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.7,
            min_hand_presence_confidence=0.7,
            min_tracking_confidence=0.6,
        )
        self.landmarker = mp_vision.HandLandmarker.create_from_options(options)

        self.create_timer(1.0 / 30.0, self.timer_callback)

    def _is_pinching(self, landmarks) -> bool:
        thumb = landmarks[4]
        index = landmarks[8]
        dist = ((thumb.x - index.x) ** 2 + (thumb.y - index.y) ** 2) ** 0.5
        return dist < PINCH_THRESHOLD

    def _draw_landmarks(self, frame, landmarks):
        h, w = frame.shape[:2]
        pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
        for a, b in HAND_CONNECTIONS:
            cv2.line(frame, pts[a], pts[b], (0, 128, 255), 2)
        for pt in pts:
            cv2.circle(frame, pt, 4, (255, 255, 255), -1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read webcam frame.')
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        timestamp_ms = int(time.time() * 1000)
        result = self.landmarker.detect_for_video(mp_image, timestamp_ms)

        left_pinch  = False
        right_pinch = False
        lift_up     = False
        lift_down   = False
        index_y_in_zone = None  # for HUD indicator

        for hand_landmarks in result.hand_landmarks:
            self._draw_landmarks(frame, hand_landmarks)

            wx = hand_landmarks[0].x   # wrist x
            ix = hand_landmarks[8].x   # index finger tip x
            iy = hand_landmarks[8].y   # index finger tip y

            if ix > LIFT_ZONE_X:
                index_y_in_zone = iy
                if iy < 0.5:
                    lift_up = True
                else:
                    lift_down = True
            elif wx > 0.5:
                if self._is_pinching(hand_landmarks):
                    right_pinch = True
            else:
                if self._is_pinching(hand_landmarks):
                    left_pinch = True

        self.cmd_vel_pub.publish(self._compute_twist(left_pinch, right_pinch))
        self.lift_pub.publish(self._compute_lift(lift_up, lift_down))

        self._draw_hud(frame, left_pinch, right_pinch, lift_up, lift_down, index_y_in_zone)
        cv2.imshow('Gesture Control', frame)
        cv2.waitKey(1)

    def _compute_twist(self, left_pinch: bool, right_pinch: bool) -> Twist:
        twist = Twist()
        if left_pinch and right_pinch:
            twist.linear.x  =  LINEAR_SPEED
            twist.angular.z =  0.0
        elif right_pinch:
            twist.linear.x  =  LINEAR_SPEED
            twist.angular.z = -ANGULAR_SPEED
        elif left_pinch:
            twist.linear.x  =  LINEAR_SPEED
            twist.angular.z =  ANGULAR_SPEED
        return twist

    def _compute_lift(self, lift_up: bool, lift_down: bool) -> Float64MultiArray:
        msg = Float64MultiArray()
        if lift_up:
            msg.data = [LIFT_EFFORT]
        elif lift_down:
            msg.data = [-LIFT_EFFORT]
        return msg

    def _draw_hud(self, frame, left_pinch, right_pinch, lift_up, lift_down, index_y_in_zone):
        h, w = frame.shape[:2]
        mid       = w // 2
        lift_x    = int(w * LIFT_ZONE_X)

        cv2.line(frame, (mid, 0), (mid, h), (200, 200, 200), 1)

        cv2.rectangle(frame, (lift_x, 0),   (w, h // 2), (0, 80, 0),   -1)
        cv2.rectangle(frame, (lift_x, h//2), (w, h),      (0, 0, 80),   -1)
        cv2.line(frame, (lift_x, 0), (lift_x, h), (180, 180, 180), 2)
        cv2.line(frame, (lift_x, h//2), (w, h//2), (180, 180, 180), 1)

        cv2.putText(frame, 'UP',   (lift_x + 6, h // 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
        cv2.putText(frame, 'DOWN', (lift_x + 6, h * 3 // 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 80, 220), 2)

        if index_y_in_zone is not None:
            iy_px = int(index_y_in_zone * h)
            cv2.line(frame, (lift_x, iy_px), (w, iy_px), (255, 255, 0), 2)

        if lift_up:
            cv2.rectangle(frame, (lift_x, 0), (w, h // 2), (0, 200, 0), 3)
        elif lift_down:
            cv2.rectangle(frame, (lift_x, h // 2), (w, h), (0, 0, 200), 3)

        cv2.putText(frame, 'LEFT ZONE',  (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        cv2.putText(frame, 'RIGHT ZONE', (mid + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        if left_pinch:
            cv2.putText(frame, 'PINCH', (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if right_pinch:
            cv2.putText(frame, 'PINCH', (mid + 10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if left_pinch and right_pinch:
            cmd, color = 'CMD: FORWARD', (0, 255, 0)
        elif right_pinch:
            cmd, color = 'CMD: FWD + TURN RIGHT', (0, 200, 255)
        elif left_pinch:
            cmd, color = 'CMD: FWD + TURN LEFT', (0, 200, 255)
        else:
            cmd, color = 'CMD: STOP', (100, 100, 100)

        cv2.putText(frame, cmd, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        if lift_up:
            lift_cmd, lift_color = 'LIFT: UP', (0, 220, 0)
        elif lift_down:
            lift_cmd, lift_color = 'LIFT: DOWN', (0, 80, 220)
        else:
            lift_cmd, lift_color = 'LIFT: STOP', (100, 100, 100)

        cv2.putText(frame, lift_cmd, (lift_x + 4, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, lift_color, 2)

def main(args=None):
    rclpy.init(args=args)
    node = HandsControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        node.landmarker.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
