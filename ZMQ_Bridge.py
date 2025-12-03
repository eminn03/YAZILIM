#!/usr/bin/env python3
"""
ZMQ Bridge Node - Jetson Side
Bu node tüm ROS topic'lerinden gelen verileri ZeroMQ üzerinden PC'ye iletir.
Pythondaki ZMQ kütüphanesinden yararlanarak wifi üzerinden görev nodelarından gelen verileri yer istasyonunda bulunan PC arayüzüne aktarır
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import zmq
import cv2
import json
import time
import numpy as np
from threading import Lock


class ZMQBridgeNode(Node):
    """
    ROS 2 topic'lerinden gelen verileri ZeroMQ PUB socket'leri üzerinden PC'ye gönderir.
    """
    
    def __init__(self):
        super().__init__('zmq_bridge_node')
        
        # Parametreleri tanımla
        self.declare_parameter('pc_ip', '192.168.1.132')
        self.declare_parameter('health_rate', 1.0)
        self.declare_parameter('jpeg_quality', 85)
        
        self.pc_ip = self.get_parameter('pc_ip').value
        self.health_rate = self.get_parameter('health_rate').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # ZeroMQ context ve socket'leri oluştur (context yerine zmq_context kullan!)
        self.zmq_context = zmq.Context()
        self.setup_zmq_publishers()
        
        # CV Bridge (ROS Image ↔ OpenCV)
        self.bridge = CvBridge()
        
        # İstatistik ve durum bilgileri
        self.stats = {
            'logitech_count': 0,
            'telemetry_count': 0,
            'realsense_rgb_count': 0,
            'realsense_depth_count': 0,
            'last_logitech_time': 0,
            'last_telemetry_time': 0,
            'last_realsense_rgb_time': 0,
            'last_realsense_depth_time': 0,
        }
        self.stats_lock = Lock()
        
        # ROS Subscribers oluştur
        self.create_ros_subscribers()
        
        # Health check timer başlat
        self.health_timer = self.create_timer(
            1.0 / self.health_rate,
            self.publish_health_status
        )
        
        self.get_logger().info('🚀 ZMQ Bridge Node başlatıldı')
        self.get_logger().info(f'📡 PC IP: {self.pc_ip}')
        self.get_logger().info('🔌 Port yapılandırması:')
        self.get_logger().info('   6000: Logitech RGB')
        self.get_logger().info('   6001: Telemetri JSON')
        self.get_logger().info('   6002: RealSense RGB')
        self.get_logger().info('   6003: RealSense Depth')
        self.get_logger().info('   6004: Health Status')
    
    def setup_zmq_publishers(self):
        """ZeroMQ PUB socket'lerini yapılandır"""
        self.zmq_sockets = {}
        
        ports = {
            'logitech': 6000,
            'telemetry': 6001,
            'realsense_rgb': 6002,
            'realsense_depth': 6003,
            'health': 6004
        }
        
        for name, port in ports.items():
            socket = self.zmq_context.socket(zmq.PUB)
            address = f"tcp://*:{port}"
            socket.bind(address)
            socket.setsockopt(zmq.SNDHWM, 10)
            self.zmq_sockets[name] = socket
            self.get_logger().info(f'✅ ZMQ PUB socket bağlandı: {address}')
    
    def create_ros_subscribers(self):
        """ROS topic'lerine abone ol"""
        
        self.logitech_sub = self.create_subscription(
            Image, '/logitech/image_raw', self.logitech_callback, 10)
        
        self.telemetry_sub = self.create_subscription(
            String, '/marmara/telemetry_json', self.telemetry_callback, 10)
        
        self.realsense_rgb_sub = self.create_subscription(
            Image, '/realsense/rgb/image_raw', self.realsense_rgb_callback, 10)
        
        self.realsense_depth_sub = self.create_subscription(
            Image, '/realsense/depth/image_rect', self.realsense_depth_callback, 10)
        
        self.get_logger().info('✅ ROS topic abonelikleri oluşturuldu')
    
    def logitech_callback(self, msg):
        """Logitech kamera görüntüsünü işle ve ZMQ'ya gönder"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, jpeg_data = cv2.imencode('.jpg', cv_image, encode_param)
            
            metadata = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'compressed_size': len(jpeg_data)
            }
            
            self.zmq_sockets['logitech'].send_multipart([
                json.dumps(metadata).encode('utf-8'),
                jpeg_data.tobytes()
            ])
            
            with self.stats_lock:
                self.stats['logitech_count'] += 1
                self.stats['last_logitech_time'] = time.time()
            
        except Exception as e:
            self.get_logger().error(f'❌ Logitech callback hatası: {e}')
    
    def telemetry_callback(self, msg):
        """Telemetri JSON'ını ZMQ'ya gönder"""
        try:
            telemetry_data = json.loads(msg.data)
            telemetry_data['bridge_timestamp'] = time.time()
            
            self.zmq_sockets['telemetry'].send_string(json.dumps(telemetry_data))
            
            with self.stats_lock:
                self.stats['telemetry_count'] += 1
                self.stats['last_telemetry_time'] = time.time()
            
        except Exception as e:
            self.get_logger().error(f'❌ Telemetri callback hatası: {e}')
    
    def realsense_rgb_callback(self, msg):
        """RealSense RGB görüntüsünü işle ve ZMQ'ya gönder"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, jpeg_data = cv2.imencode('.jpg', cv_image, encode_param)
            
            metadata = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'compressed_size': len(jpeg_data)
            }
            
            self.zmq_sockets['realsense_rgb'].send_multipart([
                json.dumps(metadata).encode('utf-8'),
                jpeg_data.tobytes()
            ])
            
            with self.stats_lock:
                self.stats['realsense_rgb_count'] += 1
                self.stats['last_realsense_rgb_time'] = time.time()
            
        except Exception as e:
            self.get_logger().error(f'❌ RealSense RGB callback hatası: {e}')
    
    def realsense_depth_callback(self, msg):
        """RealSense Depth görüntüsünü işle, renklendir ve ZMQ'ya gönder"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            depth_normalized = cv2.normalize(
                depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, jpeg_data = cv2.imencode('.jpg', depth_colormap, encode_param)
            
            metadata = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id,
                'width': msg.width,
                'height': msg.height,
                'encoding': 'depth_colormap',
                'original_encoding': msg.encoding,
                'compressed_size': len(jpeg_data)
            }
            
            self.zmq_sockets['realsense_depth'].send_multipart([
                json.dumps(metadata).encode('utf-8'),
                jpeg_data.tobytes()
            ])
            
            with self.stats_lock:
                self.stats['realsense_depth_count'] += 1
                self.stats['last_realsense_depth_time'] = time.time()
            
        except Exception as e:
            self.get_logger().error(f'❌ RealSense Depth callback hatası: {e}')
    
    def publish_health_status(self):
        """Sistem sağlık durumunu periyodik olarak yayınla"""
        try:
            current_time = time.time()
            
            with self.stats_lock:
                logitech_alive = (current_time - self.stats['last_logitech_time']) < 2.0
                telemetry_alive = (current_time - self.stats['last_telemetry_time']) < 2.0
                realsense_rgb_alive = (current_time - self.stats['last_realsense_rgb_time']) < 2.0
                realsense_depth_alive = (current_time - self.stats['last_realsense_depth_time']) < 2.0
                
                health_data = {
                    'timestamp': current_time,
                    'node_name': self.get_name(),
                    'status': 'healthy',
                    'streams': {
                        'logitech': {
                            'alive': logitech_alive,
                            'count': self.stats['logitech_count'],
                            'last_update': self.stats['last_logitech_time']
                        },
                        'telemetry': {
                            'alive': telemetry_alive,
                            'count': self.stats['telemetry_count'],
                            'last_update': self.stats['last_telemetry_time']
                        },
                        'realsense_rgb': {
                            'alive': realsense_rgb_alive,
                            'count': self.stats['realsense_rgb_count'],
                            'last_update': self.stats['last_realsense_rgb_time']
                        },
                        'realsense_depth': {
                            'alive': realsense_depth_alive,
                            'count': self.stats['realsense_depth_count'],
                            'last_update': self.stats['last_realsense_depth_time']
                        }
                    }
                }
            
            self.zmq_sockets['health'].send_string(json.dumps(health_data))
            
        except Exception as e:
            self.get_logger().error(f'❌ Health status yayın hatası: {e}')
    
    def destroy_node(self):
        """Node kapanırken ZMQ socket'lerini temizle"""
        self.get_logger().info('🛑 ZMQ Bridge Node kapatılıyor...')
        
        for name, socket in self.zmq_sockets.items():
            socket.close()
            self.get_logger().info(f'✅ {name} socket kapatıldı')
        
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZMQBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
