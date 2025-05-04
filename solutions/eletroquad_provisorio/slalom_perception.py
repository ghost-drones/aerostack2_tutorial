"""
Percepção, Slalom

• Identificação de Barras coloridas via OpenCV (image_callback)  
• Estimação da Posição 3D das Barras usando Geometria (compute_3d_points)

Inputs:  
• Parâmetros intrínsecos da câmera: `camera_info`  
• Parâmetros extrínsecos da câmera: `drone_pose` → `cam_pose`  
• Imagem crua: `image_raw`

Outputs:  
• Imagem anotada com deteções: `detected_image`  
• Poses 3D das barras: `barra_{cor}/pose`  
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

class CameraInfoSubscriber(Node):
    def __init__(self):
        super().__init__('camera_info_subscriber')

        # Parâmetros ROS
        self.declare_parameters(
            namespace='',
            parameters=[
                ('save_once', True),
                ('num_top_samples', 5),
            ]
        )
        self.save_once = self.get_parameter('save_once').get_parameter_value().bool_value
        self.num_top_samples = self.get_parameter('num_top_samples').get_parameter_value().integer_value

        # Subscrições
        self.create_subscription(CameraInfo,
                                 '/x500_px4/sensor_measurements/camera/camera_info',
                                 self.camera_info_callback, 10)
        self.create_subscription(PoseStamped,
                                 '/x500_px4/ground_truth/pose',
                                 self.pose_callback,
                                 qos_profile=QoSProfile(depth=10,
                                                        reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Image,
                                 '/x500_px4/sensor_measurements/camera/image_raw',
                                 self.image_callback, 10)

        # Publicador da imagem com deteções
        self.publisher_detected_image = self.create_publisher(Image,
                                                              '/x500_px4/ghost/detected_image', 10)

        # Estado interno
        self.fx = self.fy = self.cx = self.cy = None
        self.saved_intrinsics = False
        self.x_camera = self.y_camera = self.z_camera = None
        self.qx = self.qy = self.qz = self.qw = None

        self.bridge = CvBridge()
        self.get_logger().info('Nó inicializado: aguardando CameraInfo, Pose e Image...')

    def camera_info_callback(self, msg: CameraInfo):
        if self.save_once and self.saved_intrinsics:
            return

        # Extrai intrínsecos
        K = np.array(msg.k).reshape((3, 3))
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]
        self.inv_K = np.linalg.inv(K)

        self.saved_intrinsics = True
        self.get_logger().info(f'Intrínsecos salvos: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                               f'cx={self.cx:.2f}, cy={self.cy:.2f}')

    def quaternion_from_euler(self, roll, pitch, yaw):
        return R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

    def quaternion_multiply(self, q1, q2):
        return (R.from_quat(q1) * R.from_quat(q2)).as_quat()

    def pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        # Correção fixa de offset da câmera
        self.x_camera = p.x + 0.04
        self.y_camera = p.y
        self.z_camera = p.z - 0.1

        q_raw = [msg.pose.orientation.x,
                 msg.pose.orientation.y,
                 msg.pose.orientation.z,
                 msg.pose.orientation.w]
        q_corr = self.quaternion_from_euler(-1.57, 0.0, -1.57)
        self.qx, self.qy, self.qz, self.qw = self.quaternion_multiply(q_corr, q_raw)

    def image_callback(self, msg: Image):
        if not self.saved_intrinsics:
            self.get_logger().warn('Aguardando intrínsecos da câmera...')
            return
        if None in (self.x_camera, self.y_camera, self.z_camera):
            self.get_logger().warn('Aguardando pose da câmera...')
            return

        # Converte para OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Falha na conversão da imagem: {e}')
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        color_ranges = {
            'VERMELHA': ((0, 100, 100), (10, 255, 255)),
            'AZUL':     ((110, 100, 100), (130, 255, 255)),
            'ROSA':     ((140, 100, 100), (160, 255, 255)),
            'PRETA':    ((0, 0, 0), (180, 255, 50)),
        }

        detections = []
        for name, (lo, hi) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                detections.append((c, name))

        # Pré-cálculo de rotação e origem do raio
        R_wc = R.from_quat([self.qx, self.qy, self.qz, self.qw]).as_matrix()
        origin = np.array([self.x_camera, self.y_camera, self.z_camera])

        for idx, (cnt, color) in enumerate(detections):
            x, y, w, h = cv2.boundingRect(cnt)

            if h <= 2 * w or h <= 30 or y < 5:
                continue

            # Gera amostras na aresta superior
            us = np.linspace(x, x + w, self.num_top_samples + 2)[1:-1]
            vs = np.full_like(us, y, dtype=float)
            pts_world = self.compute_3d_points(us, vs, R_wc, origin, Z_plane=2.5)

            if len(pts_world) == 0:
                self.get_logger().warn(f'ID {idx}: falha em todas amostras 3D.')
                continue

            P_avg = np.mean(pts_world, axis=0)
            self.get_logger().info(
                f'ID {idx}: Cor={color} | TopCenter 3D ≈ [{P_avg[0]:.3f}, {P_avg[1]:.3f}, {P_avg[2]:.3f}]'
            )

            # Desenhos
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cx, cy = int(x + w / 2), int(y)
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(img, f'ID {idx}-{color}', (cx - 40, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Publica resultado
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_detected_image.publish(out_msg)

    def compute_3d_points(self, us, vs, R_wc, origin, Z_plane):
        """
        Recebe arrays us, vs e retorna lista de pontos 3D no plano Z=Z_plane.
        """
        pts = []
        for u, v in zip(us, vs):
            # raio na câmera
            pix = np.array([u, v, 1.0])
            dir_cam = self.inv_K.dot(pix)
            dir_cam /= np.linalg.norm(dir_cam)
            # para o mundo
            dir_w = R_wc.dot(dir_cam)
            oz, dz = origin[2], dir_w[2]
            if abs(dz) < 1e-6:
                continue
            t = (Z_plane - oz) / dz
            pts.append(origin + t * dir_w)
        return pts

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
