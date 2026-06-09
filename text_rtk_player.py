import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class TextRTKPlayer(Node):
    def __init__(self, txt_file):
        super().__init__('text_rtk_player')
        
        # 🌟 开启仿真时间，强制让它跟随 ros2 bag play 的进度条
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(NavSatFix, '/rtk_fix', 10)
        self.gnss_msgs = []
        
        self.get_logger().info('🚀 正在绕开底层封锁，直接从 TXT 文本加载高精度真值...')
        
        # 解析你之前下载的那个 TXT 文件 (这里默认你把它改名叫 span_gt.txt 了)
        valid_count = 0
        with open(txt_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('UTC') or line.startswith('(sec)'):
                    continue
                    
                parts = line.split()
                if len(parts) < 10:
                    continue
                    
                try:
                    # 1. 解析时间戳
                    timestamp = float(parts[0])
                    
                    # 2. 解析度分秒并转为十进制经纬度
                    lat_deg, lat_min, lat_sec = float(parts[3]), float(parts[4]), float(parts[5])
                    lon_deg, lon_min, lon_sec = float(parts[6]), float(parts[7]), float(parts[8])
                    
                    lat = lat_deg + lat_min / 60.0 + lat_sec / 3600.0
                    lon = lon_deg + lon_min / 60.0 + lon_sec / 3600.0
                    alt = float(parts[9])
                    
                    # 3. 组装成标准的 ROS 2 消息
                    fix = NavSatFix()
                    fix.header.stamp.sec = int(timestamp)
                    fix.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
                    fix.header.frame_id = 'gnss'
                    
                    fix.latitude = lat
                    fix.longitude = lon
                    fix.altitude = alt
                    
                    # 填入高精度设备的经验协方差 (0.05m 级别)
                    fix.position_covariance[0] = 0.05 ** 2
                    fix.position_covariance[4] = 0.05 ** 2
                    fix.position_covariance[8] = 0.10 ** 2
                    fix.status.status = 0
                    
                    self.gnss_msgs.append((timestamp, fix))
                    valid_count += 1
                except ValueError:
                    continue
                    
        self.get_logger().info(f'✅ 成功从文本加载了 {valid_count} 条无暇的 RTK 弹药！等待时钟同步...')
        
        # 定时器：以 50Hz 的频率检查系统时钟
        self.current_idx = 0
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if self.current_idx >= len(self.gnss_msgs):
            return
            
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
            
        now_sec = now_ns / 1e9
        
        # 一旦 ros2 bag play 的时间推进到了当前 GNSS 点的时间，发射！
        while self.current_idx < len(self.gnss_msgs) and now_sec >= self.gnss_msgs[self.current_idx][0]:
            self.pub.publish(self.gnss_msgs[self.current_idx][1])
            self.current_idx += 1

def main():
    rclpy.init()
    # 🚨 注意：把你之前那个“新建 文本文档.txt”放到同目录下，并填入它的正确文件名
    txt_path = 'span_gt.txt' 
    node = TextRTKPlayer(txt_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
