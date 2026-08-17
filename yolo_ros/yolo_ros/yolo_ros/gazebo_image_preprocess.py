# Gazebo Image Preprocessing Node
# Gazebo simülasyon görüntülerini gerçek dünyaya yaklaştırmak için ön işleme yapar.
# Ana sorun: Gazebo lineer renk uzayında render eder, gerçek kameralar sRGB (gamma) kullanır.

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class GazeboImagePreprocessNode(Node):
    """
    Gazebo kamera görüntüsünü YOLO modeli için ön işlemden geçiren ROS2 node.

    Yapılan işlemler (sırasıyla):
    1. Renk kanalı düzeltme (RGB → BGR)
    2. Gamma düzeltmesi (lineer → sRGB) — EN KRİTİK
    3. CLAHE histogram eşitleme (kontrast normalizasyonu)
    4. Hafif Gaussian blur (simülasyon keskin hatlarını yumuşatma)
    5. Opsiyonel gürültü ekleme (domain gap azaltma)
    """

    def __init__(self):
        super().__init__("gazebo_image_preprocess")

        # Parametreler
        self.declare_parameter("input_topic", "/image")
        self.declare_parameter("output_topic", "/sim/image")
        self.declare_parameter("enable_gamma", True)
        self.declare_parameter("gamma_value", 0.4545)  # 1/2.2 = sRGB gamma
        self.declare_parameter("enable_clahe", True)
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_grid_size", 8)
        self.declare_parameter("enable_blur", True)
        self.declare_parameter("blur_kernel_size", 3)
        self.declare_parameter("enable_noise", False)
        self.declare_parameter("noise_sigma", 3.0)
        self.declare_parameter("enable_brightness", True)
        self.declare_parameter("brightness_alpha", 1.1)  # kontrast çarpanı
        self.declare_parameter("brightness_beta", 10.0)  # parlaklık ekleme
        self.declare_parameter("image_reliability", 1)  # BEST_EFFORT

        # Parametreleri oku
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.enable_gamma = self.get_parameter("enable_gamma").value
        self.gamma_value = self.get_parameter("gamma_value").value
        self.enable_clahe = self.get_parameter("enable_clahe").value
        self.clahe_clip_limit = self.get_parameter("clahe_clip_limit").value
        self.clahe_grid_size = self.get_parameter("clahe_grid_size").value
        self.enable_blur = self.get_parameter("enable_blur").value
        self.blur_kernel_size = self.get_parameter("blur_kernel_size").value
        self.enable_noise = self.get_parameter("enable_noise").value
        self.noise_sigma = self.get_parameter("noise_sigma").value
        self.enable_brightness = self.get_parameter("enable_brightness").value
        self.brightness_alpha = self.get_parameter("brightness_alpha").value
        self.brightness_beta = self.get_parameter("brightness_beta").value
        reliability = self.get_parameter("image_reliability").value

        # QoS
        qos = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # CLAHE objesi
        self.clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip_limit,
            tileGridSize=(self.clahe_grid_size, self.clahe_grid_size),
        )

        # Gamma LUT (lookup table) — hızlı uygulama için
        self._build_gamma_lut(self.gamma_value)

        self.cv_bridge = CvBridge()
        self._encoding_logged = False

        # Sub & Pub
        self.sub = self.create_subscription(Image, input_topic, self.image_cb, qos)
        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f"Gazebo Image Preprocess başlatıldı: {input_topic} → {output_topic}"
        )
        self.get_logger().info(
            f"  Gamma: {self.enable_gamma} (γ={self.gamma_value:.4f}), "
            f"CLAHE: {self.enable_clahe}, Blur: {self.enable_blur}, "
            f"Noise: {self.enable_noise}, Brightness: {self.enable_brightness}"
        )

    def _build_gamma_lut(self, gamma):
        """Gamma düzeltmesi için lookup table oluştur (performans için)."""
        inv_gamma = 1.0 / gamma if gamma > 0 else 1.0
        self.gamma_lut = np.array(
            [((i / 255.0) ** gamma) * 255 for i in np.arange(256)]
        ).astype("uint8")
        self.get_logger().info(f"Gamma LUT oluşturuldu: γ = {gamma:.4f}")

    def image_cb(self, msg: Image):
        # Encoding bilgisini bir kere logla
        if not self._encoding_logged:
            self.get_logger().info(
                f"Gelen görüntü encoding: '{msg.encoding}', "
                f"boyut: {msg.width}x{msg.height}"
            )
            self._encoding_logged = True

        # --- 1. Renk Kanalı Düzeltmesi ---
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if msg.encoding in ("rgb8", "RGB8"):
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f"Görüntü çevirme hatası: {e}")
            return

        # --- 2. Gamma Düzeltmesi (Lineer → sRGB) ---
        # Gazebo lineer renk uzayında render eder.
        # Gerçek kameralar sRGB gamma (~2.2) uygular.
        # Bu adım simülasyon görüntüsünü gerçek kameraya benzetir.
        if self.enable_gamma:
            cv_image = cv2.LUT(cv_image, self.gamma_lut)

        # --- 3. Parlaklık/Kontrast Ayarı ---
        if self.enable_brightness:
            cv_image = cv2.convertScaleAbs(
                cv_image,
                alpha=self.brightness_alpha,
                beta=self.brightness_beta,
            )

        # --- 4. CLAHE Histogram Eşitleme ---
        if self.enable_clahe:
            lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
            cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # --- 5. Gaussian Blur ---
        if self.enable_blur:
            k = self.blur_kernel_size
            if k % 2 == 0:
                k += 1
            cv_image = cv2.GaussianBlur(cv_image, (k, k), 0)

        # --- 6. Gürültü Ekleme ---
        if self.enable_noise and self.noise_sigma > 0:
            noise = np.random.normal(0, self.noise_sigma, cv_image.shape).astype(np.float32)
            cv_image = np.clip(cv_image.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        # Yayınla
        out_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = GazeboImagePreprocessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()