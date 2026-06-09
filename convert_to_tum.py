import sys
import pyproj
from scipy.spatial.transform import Rotation as R


def convert_span_to_tum(input_file, output_file):
    # 初始化 UTM 投影 (香港位于 UTM 50N 投影带)
    proj = pyproj.Proj(proj='utm', zone=50, ellps='WGS84', preserve_units=False)

    origin_x, origin_y, origin_z = None, None, None
    valid_count = 0

    with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
        for line in f_in:
            line = line.strip()
            # 跳过表头和空行
            if not line or line.startswith('UTC') or line.startswith('(sec)'):
                continue

            parts = line.split()
            # 确保行数据完整 (至少包含到 Q 状态码)
            if len(parts) < 20:
                continue

            try:
                # 1. 解析时间戳
                timestamp = float(parts[0])

                # 2. 解析 度分秒 (D M S) 经纬度并转为十进制
                lat_deg, lat_min, lat_sec = float(parts[3]), float(parts[4]), float(parts[5])
                lon_deg, lon_min, lon_sec = float(parts[6]), float(parts[7]), float(parts[8])

                lat = lat_deg + lat_min / 60.0 + lat_sec / 3600.0
                lon = lon_deg + lon_min / 60.0 + lon_sec / 3600.0
                alt = float(parts[9])

                # 3. 经纬度投影到 UTM 平面坐标系
                x, y = proj(lon, lat)
                z = alt

                # 扣除起点坐标，将其转换为以起点为 (0,0,0) 的局部 ENU 坐标系
                if origin_x is None:
                    origin_x, origin_y, origin_z = x, y, z

                local_x = x - origin_x
                local_y = y - origin_y
                local_z = z - origin_z

                # 4. 解析欧拉角 (Roll, Pitch, Heading)
                roll = float(parts[16])
                pitch = float(parts[17])
                heading = float(parts[18])

                # SPAN 的 Heading 是以正北为 0 度，顺时针旋转 (Azimuth)。
                # 而标准的 ENU 坐标系，Yaw 是以正东为 0 度，逆时针旋转。因此需要转换：
                yaw = 90.0 - heading

                # 将欧拉角 (Z-Y-X 顺序) 转换为 四元数
                rot = R.from_euler('ZYX', [yaw, pitch, roll], degrees=True)
                qx, qy, qz, qw = rot.as_quat()

                # 5. 写入 TUM 格式
                f_out.write(
                    f"{timestamp:.6f} {local_x:.6f} {local_y:.6f} {local_z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
                valid_count += 1

            except ValueError as e:
                continue

    print(f"✅ 转换成功！共处理 {valid_count} 帧位姿。")
    print(f"✅ 真值已保存至: {output_file}")


if __name__ == '__main__':
    convert_span_to_tum('span_gt.txt', 'gt_tum.txt')