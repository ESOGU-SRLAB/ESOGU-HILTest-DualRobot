#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from std_srvs.srv import Trigger
import csv
from datetime import datetime
import threading
import sys
from pynput import keyboard

class TCPRecorder(Node):
    def __init__(self):
        super().__init__('tcp_recorder')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.record_count = 0
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'tcp_positions_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['record_id', 'timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        
        # Keyboard listener başlat
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        
        self.get_logger().info(f'TCP Recorder ready!')
        self.get_logger().info(f'Saving to: {self.csv_filename}')
        self.get_logger().info(f'Press "h" to record TCP position')
    
    def on_key_press(self, key):
        try:
            if key.char == 'h':
                self.record_tcp()
        except AttributeError:
            pass
    
    def record_tcp(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'ur10e_tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.record_count += 1
            
            position = [
                self.record_count,
                datetime.now().isoformat(),
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
            self.csv_writer.writerow(position)
            self.csv_file.flush()
            
            self.get_logger().info(
                f'✓ Record #{self.record_count}/50 saved\n'
                f'  Position: x={position[2]:.3f}, y={position[3]:.3f}, z={position[4]:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
    
    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        if hasattr(self, 'listener'):
            self.listener.stop()

def main(args=None):
    rclpy.init(args=args)
    recorder = TCPRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()