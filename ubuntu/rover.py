#!/usr/bin/env python3
"""
ESP32 Rover: Manuel + Otonom sürüş + Canlı LiDAR + Cartographer haritalama

Gereksinimler (haritalama sekmesi için, ROS 2 ortamı source edilmiş olmalı):
    sudo apt install ros-$ROS_DISTRO-cartographer-ros ros-$ROS_DISTRO-nav2-map-server
    source /opt/ros/$ROS_DISTRO/setup.bash
    python3 rover.py

ROS 2 yoksa script yine çalışır; sadece "Harita" sekmesi devre dışı kalır.
"""
import heapq
import json
import math
import os
import signal
import socket
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

# ---- ROS 2 (opsiyonel) ----
try:
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    ROS_OK = True
    ROS_ERR = ""
except Exception as _e:  # ROS kurulu değil / source edilmemiş
    ROS_OK = False
    ROS_ERR = str(_e)

# ---- Pillow (opsiyonel, pürüzsüz harita çizimi için) ----
try:
    from PIL import Image, ImageTk
    PIL_OK = True
except Exception:
    PIL_OK = False

# =========================================================
# AĞ VE ESP32 AYARLARI
# =========================================================
ESP32_IP = "192.168.1.3"
MOTOR_PORT = 4210
LIDAR_PORT = 4211
BIND_HOST = "0.0.0.0"

MAX_RANGE_M = 6.0           # Radar ekranı menzili
POINT_LIFETIME = 0.5        # Noktaların ekranda kalma süresi (sn)

# =========================================================
# CARTOGRAPHER / LIDAR -> ROS AYARLARI
# =========================================================
# LiDAR ham açı indeksi, robota YUKARIDAN bakınca saat yönünde mi artıyor?
# Emin değilseniz harita sekmesindeki 'Yön Testi' bunu ölçüp ~/.rover_cartographer/calib.json'a kaydeder.
# Bu ayar radar ekranını, otonom modu, hedefe gidişi ve ROS taramasını BİRLİKTE etkiler.
LIDAR_ANGLES_CLOCKWISE = True
SCAN_RANGE_MIN = 0.06
SCAN_RANGE_MAX = 5.5
MAP_RESOLUTION = 0.03       # metre / hücre (küçüldükçe daha detaylı)
CFG_DIR = os.path.join(os.path.expanduser("~"), ".rover_cartographer")
LUA_NAME = "rover_2d.lua"

# LiDAR-only Cartographer: odometri/IMU yok; harita yine de zamanla drift edebilir.
LUA_CONFIG = r'''
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 4.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.10
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35

POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

return options
'''

# =========================================================
# OTONOM: HIZ AYARLARI (PWM)
# =========================================================
CRUISE_SPEED = 100
TURN_SPEED = 200
UNSTUCK_PWM = 175

# =========================================================
# OTONOM: EŞİK MESAFELERİ (Metre)
# =========================================================
FRONT_BLOCK_DIST = 0.35
WIDE_BLOCK_DIST = 0.22
TARGET_CLEAR_DIST = 0.80

REAR_CLEAR_DIST = 0.50      # Arkada bu mesafeden yakın bir şey varsa GERİ GİTME
REAR_SPREAD = 45            # Arka ±45° taranır
REAR_INVALID_BLOCKS = True  # Arka okumalar geçersizse (çok yakın nesne) dolu say

# =========================================================
# OTONOM: DÖNÜŞ AYARLARI
# =========================================================
MIN_TURN_TIME = 0.30
TURN_TIMEOUT = 5.0

# =========================================================
# OTONOM: SIKIŞMA (PATİNAJ) TESPİT AYARLARI
# =========================================================
STALL_CHECK_INTERVAL = 0.7
STALL_THRESHOLD_M = 0.03
STALL_CONFIRM = 3
STALL_GRACE = 0.8
STALL_NEAR_DIST = 0.60
UNSTUCK_BACK_TIME = 0.6
UNSTUCK_SPIN_TIME = 0.5

FRONT_RAW = 60              # LiDAR ham açısında robotun burnunun bulunduğu açı
# =========================================================
# HEDEFE GİDİŞ AYARLARI (haritada tıklayarak)
# =========================================================
ROBOT_RADIUS_M = 0.15       # Robotun merkezden en uzak noktasına kadar yarıçapı (KENDİ ROBOTUNUZA GÖRE AYARLAYIN)
SAFETY_MARGIN_M = 0.10      # Engellerden ekstra güvenlik payı
NAV_SOFT_ZONE_M = 0.25      # Bu bölgede planlayıcı duvardan uzak durmayı tercih eder
NAV_SOFT_COST = 4.0
NAV_PLAN_RES = 0.06         # Planlama ızgarası (m)
NAV_SPEED = 130             # İleri sürüş PWM
NAV_SPIN_PWM = 150          # Hedefe dönerken daha kontrollü yerinde dönüş
NAV_MIN_PWM = 100           # Motorların kalkış eşiği (gerekirse 100-120 arası kalibre edin)
NAV_BOOST_S = 1.0           # Komut var ama robot kıpırdamıyorsa bu sürede bir PWM artırılır
NAV_BOOST_STEP = 20         # Her adımda eklenen PWM
NAV_MAX_BOOST = 70          # En fazla eklenebilecek PWM
NAV_SPIN_ERR = 0.35         # rad (~20°): daha küçük hata kalana kadar önce yönünü düzelt
NAV_K_STEER = 60.0          # PWM / rad: daha yumuşak düzeltme, daha az yalpalama
NAV_K_REPEL = 250.0         # Yan duvara yaklaşınca itme kazancı
NAV_SIDE_CLEAR = 0.25       # Yan mesafe bundan azsa uzaklaş (m)
NAV_LOOKAHEAD_M = 0.50       # Daha ilerideki yol noktasını izle; zikzakları azaltır
NAV_TURN_SIGN = -1          # Araç hedef yönünün tersine dönüyorsa -1; düzelmezse +1 deneyin ('Yön Testi' bunu otomatik ayarlar)
NAV_SPIN_ON_S = 0.4         # Hedefe dönerken kısa darbeler: dönüş süresi (sn)
NAV_SPIN_OFF_S = 0.2        # ... ve duraklama (sn): Cartographer konumu oturtsun. 0 = sürekli dön
NAV_MIN_PKTS = 60           # Son turda bundan az LiDAR paketi geldiyse (kayıp) hareket etme, bekle
NAV_GOAL_TOL_M = 0.15
NAV_STUCK_S = 2.5
NAV_TIMEOUT_S = 180
NAV_MAX_REPLANS = 6

# =========================================================
# BAĞLANTI GÜVENLİĞİ
# =========================================================
HEARTBEAT_S = 0.1           # Son hız komutu bu aralıkla tekrar gönderilir (kayıp paketlere karşı)
LINK_WARN_S = 0.4           # LiDAR verisi bu süredir gelmiyorsa bağlantı 'zayıf'
LINK_LOST_S = 1.5           # ... bu süredir gelmiyorsa 'kopuk'
STOP_ON_LINK_LOSS = True    # Bağlantı kopuksa robotu durdur (LiDAR en az bir kez geldiyse)

REJECT_WEAK = False         # True: Neato 'zayıf sinyal' bayrağı olan noktaları ele (arayüzden de değişir)

# 'Yön Testi' sonuçları burada saklanır ve dosyadaki sabitlerin yerine geçer
CALIB_FILE = os.path.join(CFG_DIR, "calib.json")

def load_calib():
    global LIDAR_ANGLES_CLOCKWISE, FRONT_RAW, NAV_TURN_SIGN
    try:
        with open(CALIB_FILE) as f:
            d = json.load(f)
        LIDAR_ANGLES_CLOCKWISE = bool(d.get("LIDAR_ANGLES_CLOCKWISE", LIDAR_ANGLES_CLOCKWISE))
        FRONT_RAW = int(d.get("FRONT_RAW", FRONT_RAW)) % 360
        NAV_TURN_SIGN = 1 if int(d.get("NAV_TURN_SIGN", NAV_TURN_SIGN)) >= 0 else -1
    except Exception:
        pass

def save_calib():
    try:
        os.makedirs(CFG_DIR, exist_ok=True)
        with open(CALIB_FILE, "w") as f:
            json.dump({"LIDAR_ANGLES_CLOCKWISE": LIDAR_ANGLES_CLOCKWISE,
                       "FRONT_RAW": FRONT_RAW, "NAV_TURN_SIGN": NAV_TURN_SIGN}, f)
    except Exception as e:
        print(f"Kalibrasyon kaydedilemedi: {e}")

load_calib()

def raw_to_gui(ang):
    """LiDAR ham açısı -> robot burnundan SAAT YÖNÜNDE ölçülen açı (radar/otonom/harita için ortak)."""
    return (ang - FRONT_RAW) % 360 if LIDAR_ANGLES_CLOCKWISE else (FRONT_RAW - ang) % 360

def gui_to_raw(g):
    return (g + FRONT_RAW) % 360 if LIDAR_ANGLES_CLOCKWISE else (FRONT_RAW - g) % 360

# =========================================================
# GLOBAL DEĞİŞKENLER
# =========================================================
motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

speed = 100
current_left = 0
current_right = 0
last_cmd_text = "Waiting for ESP32..."
pressed_keys = set()

is_autonomous = False
robot_state = 'SURUS'       # 'SURUS', 'DONUS', 'KURTULMA'
turn_dir = 1                # 1 = sol, -1 = sağ
turn_start_time = 0.0

last_stall_check_time = time.time()
prev_scan_snapshot = None
stall_streak = 0
stall_grace_until = 0.0
stuck_display_until = 0.0
unstuck_until_time = 0.0
unstuck_mode = 'back'
unstuck_count = 0

ranges = [float('inf')] * 360
lidar_points = {}
lock = threading.Lock()
running = True
last_send_time = 0.0
lidar_seen = False
scan_stats = {"pkts": 0, "valid": 0, "invalid": 0, "weak": 0, "badchk": 0}
last_scan_time = time.time()

ros_bridge = None           # RosBridge örneği (ROS varsa)
carto_procs = []            # Cartographer süreçleri
carto_logs = []
trail = []                  # Harita üzerinde robot izi [(x, y)]

# =========================================================
# ROS 2 KÖPRÜSÜ: /scan yayınlar, /map ve TF dinler
# =========================================================
if ROS_OK:
    class RosBridge:
        def __init__(self):
            rclpy.init()
            self.node = rclpy.create_node("rover_ui_bridge")
            self.scan_pub = self.node.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.node.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
            self._lock = threading.Lock()
            self._map = None
            self._map_ver = 0
            self._last_scan_t = None
            self.scan_count = 0
            threading.Thread(target=self._spin, daemon=True).start()

        def _spin(self):
            try:
                rclpy.spin(self.node)
            except Exception:
                pass

        def publish_scan(self, ranges_ccw):
            """ranges_ccw: 360 elemanlı, burundan başlayıp saat yönünün tersine giden mesafeler."""
            t = time.time()
            scan_time = 0.2 if self._last_scan_t is None else min(0.5, max(0.1, t - self._last_scan_t))
            self._last_scan_t = t

            end_ns = self.node.get_clock().now().nanoseconds
            start_ns = end_ns - int(scan_time * 1e9)

            msg = LaserScan()
            msg.header.stamp = Time(nanoseconds=start_ns).to_msg()
            msg.header.frame_id = "base_link"
            msg.angle_min = 0.0
            msg.angle_increment = 2.0 * math.pi / 360.0
            msg.angle_max = msg.angle_increment * 359
            msg.scan_time = float(scan_time)
            msg.time_increment = float(scan_time / 360.0)
            msg.range_min = SCAN_RANGE_MIN
            msg.range_max = SCAN_RANGE_MAX
            msg.ranges = [float(x) for x in ranges_ccw]
            self.scan_pub.publish(msg)
            self.scan_count += 1

        def _on_map(self, msg):
            w, h = msg.info.width, msg.info.height
            if w == 0 or h == 0:
                return
            arr = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
            gray = np.full((h, w), 205, dtype=np.uint8)          # bilinmeyen: gri
            known = arr >= 0
            # 0 (boş) -> beyaz, 100 (dolu) -> siyah, aradaki olasılıklar gri tonu
            gray[known] = (254 - np.clip(arr[known], 0, 100).astype(np.float32) * 2.54).astype(np.uint8)
            img = np.ascontiguousarray(np.repeat(gray[::-1][:, :, None], 3, axis=2))  # kuzey yukarı
            with self._lock:
                self._grid = (arr, w, h, msg.info.resolution,
                              msg.info.origin.position.x, msg.info.origin.position.y)
                self._map_ver += 1
                self._map = (self._map_ver, w, h, msg.info.resolution,
                             msg.info.origin.position.x, msg.info.origin.position.y, img)

        def get_map(self):
            with self._lock:
                return self._map

        def get_grid(self):
            with self._lock:
                return getattr(self, "_grid", None)

        def get_pose(self):
            try:
                tr = self.tf_buffer.lookup_transform("map", "base_link", Time())
            except Exception:
                return None
            try:
                age = (self.node.get_clock().now() - Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
                if abs(age) > 1.5:      # eski (bayat) konum
                    return None
            except Exception:
                pass
            q = tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return tr.transform.translation.x, tr.transform.translation.y, yaw

        def shutdown(self):
            try:
                rclpy.shutdown()
            except Exception:
                pass

# =========================================================
# CARTOGRAPHER SÜREÇ YÖNETİMİ
# =========================================================
def mapping_running():
    return bool(carto_procs) and all(p.poll() is None for p in carto_procs)

def start_mapping():
    if mapping_running():
        return True, "Cartographer is already running"
    os.makedirs(CFG_DIR, exist_ok=True)
    with open(os.path.join(CFG_DIR, LUA_NAME), "w") as f:
        f.write(LUA_CONFIG)

    cmds = [
        ["ros2", "run", "cartographer_ros", "cartographer_node",
         "-configuration_directory", CFG_DIR, "-configuration_basename", LUA_NAME],
        ["ros2", "run", "cartographer_ros", "cartographer_occupancy_grid_node",
         "-resolution", str(MAP_RESOLUTION), "-publish_period_sec", "1.0"],
    ]
    stop_mapping()
    try:
        for i, cmd in enumerate(cmds):
            log = open(os.path.join(CFG_DIR, f"cartographer_{i}.log"), "w")
            carto_logs.append(log)
            carto_procs.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                                start_new_session=True))
    except FileNotFoundError:
        stop_mapping()
        return False, "'ros2' command not found (source your ROS 2 environment)"
    trail.clear()
    return True, "Cartographer started"

def stop_mapping():
    for p in carto_procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
    for p in carto_procs:
        try:
            p.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
    carto_procs.clear()
    for log in carto_logs:
        try:
            log.close()
        except Exception:
            pass
    carto_logs.clear()

# =========================================================
# MOTOR ÇIKIŞI
# =========================================================
def send_velocity(left_spd, right_spd):
    """Tk widget'larına dokunmaz, her thread'den güvenle çağrılabilir."""
    global current_left, current_right, last_cmd_text, last_send_time
    left_spd = int(max(-255, min(255, left_spd)))
    right_spd = int(max(-255, min(255, right_spd)))
    current_left, current_right = left_spd, right_spd

    msg = f"V:{left_spd},{right_spd}"
    try:
        motor_sock.sendto(msg.encode(), (ESP32_IP, MOTOR_PORT))
        last_send_time = time.time()
        last_cmd_text = f"ESP32 → {msg}"
    except Exception as e:
        last_cmd_text = f"Error: {e}"

def stop():
    send_velocity(0, 0)

def stop_hard():
    """Önemli durdurmalarda stop komutunu üst üste 3 kez gönderir (UDP paket kaybına karşı)."""
    for _ in range(3):
        send_velocity(0, 0)

def motor_keepalive():
    """Son komutu düzenli tekrarlar. Durma komutu kaybolsa bile bir sonraki paket robotu durdurur.
    Arayüz donarsa bu da durur; ESP32 tarafındaki watchdog robotu böylece kendiliğinden durdurur."""
    if time.time() - last_send_time >= HEARTBEAT_S:
        send_velocity(current_left, current_right)

def spin(direction, pwm):
    """direction: 1 = sola, -1 = sağa (yerinde)"""
    if direction == 1:
        send_velocity(-pwm, pwm)
    else:
        send_velocity(pwm, -pwm)

# =========================================================
# MANUEL KONTROL
# =========================================================
def manual_guard():
    """Manuel komut verilebilir mi? Hedefe gidiş/yön testi sürüyorsa iptal eder (manuel müdahale önceliklidir)."""
    if is_autonomous:
        return False
    if nav["active"]:
        nav_cancel("Manual override -> navigation cancelled")
    if calib["active"]:
        calib_finish("Direction test cancelled (manual override)")
    return True

def forward():
    if manual_guard(): send_velocity(speed, speed)

def backward():
    if manual_guard(): send_velocity(-speed, -speed)

def left():
    if manual_guard(): send_velocity(-speed, speed)

def right():
    if manual_guard(): send_velocity(speed, -speed)

def manual_stop():
    if manual_guard(): stop_hard()

MOVE_KEYS = {"w", "a", "s", "d", "up", "down", "left", "right"}

def key_press(event):
    k = event.keysym.lower()
    if k == "escape":
        if nav["active"]:
            nav_cancel("Navigation cancelled (ESC)")
        elif calib["active"]:
            calib_finish("Direction test cancelled (ESC)")
        elif is_autonomous:
            toggle_mode()
        return
    if is_autonomous:
        return
    if nav["active"] or calib["active"]:
        if k in MOVE_KEYS:
            if nav["active"]:
                nav_cancel("Manual override -> navigation cancelled")
            else:
                calib_finish("Direction test cancelled (manual override)")
        else:
            return
    pressed_keys.add(k)
    update_movement()

def key_release(event):
    if is_autonomous or nav["active"] or calib["active"]:
        return
    pressed_keys.discard(event.keysym.lower())
    update_movement()

def update_movement():
    f = "w" in pressed_keys or "up" in pressed_keys
    b = "s" in pressed_keys or "down" in pressed_keys
    l = "a" in pressed_keys or "left" in pressed_keys
    r = "d" in pressed_keys or "right" in pressed_keys

    lv, rv = 0, 0
    if f and not b: lv, rv = speed, speed
    elif b and not f: lv, rv = -speed, -speed

    if l and not r: lv -= speed; rv += speed
    elif r and not l: lv += speed; rv -= speed

    send_velocity(lv, rv)

def speed_changed(val):
    global speed
    speed = int(float(val))
    speed_label.config(text=f"Speed: {speed}")

# =========================================================
# LIDAR ALICI & CHECKSUM
# =========================================================
def neato_checksum_ok(pkt):
    chk32 = 0
    for i in range(0, 20, 2):
        word = pkt[i] | (pkt[i + 1] << 8)
        chk32 = (chk32 << 1) + word
    chk32 = (chk32 & 0x7FFF) + (chk32 >> 15)
    chk32 = (chk32 & 0x7FFF) + (chk32 >> 15)
    expected = pkt[20] | (pkt[21] << 8)
    return (chk32 & 0xFFFF) == expected

def lidar_receiver():
    global last_scan_time, lidar_seen
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((BIND_HOST, LIDAR_PORT))
    sock.settimeout(0.5)

    # Tam tur (360°) birleştirme: ROS /scan için
    scan_buf = [float('inf')] * 360
    scan_pkts = 0
    last_idx = -1
    sc_valid = sc_inv = sc_weak = sc_badchk = 0

    while running:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break

        now = time.time()
        new_batch = {}
        for offset in range(0, len(data), 22):
            pkt = data[offset:offset + 22]
            if len(pkt) < 22 or pkt[0] != 0xFA or not (0xA0 <= pkt[1] <= 0xF9):
                continue
            if not neato_checksum_ok(pkt):
                sc_badchk += 1
                continue

            idx = pkt[1] - 0xA0
            # Paket indeksi başa sardıysa bir tur tamamlanmıştır -> yayınla
            if idx <= last_idx:
                if ros_bridge is not None and scan_pkts >= 70:
                    try:
                        ros_bridge.publish_scan(scan_buf)
                    except Exception as e:
                        print(f"Scan yayın hatası: {e}")
                scan_stats.update(pkts=scan_pkts, valid=sc_valid, invalid=sc_inv,
                                  weak=sc_weak, badchk=sc_badchk)
                scan_buf = [float('inf')] * 360
                scan_pkts = 0
                sc_valid = sc_inv = sc_weak = sc_badchk = 0
            last_idx = idx
            scan_pkts += 1

            base_angle = idx * 4
            with lock:
                last_scan_time = now
                lidar_seen = True
                for i in range(4):
                    p = 4 + (i * 4)
                    invalid = bool(pkt[p + 1] & 0x80)
                    weak = bool(pkt[p + 1] & 0x40)
                    dist = ((pkt[p + 1] & 0x3F) << 8) | pkt[p]
                    ang = (base_angle + i) % 360
                    gui_ang = raw_to_gui(ang)
                    ros_idx = (-gui_ang) % 360       # ROS: saat yönünün tersi (CCW) pozitif

                    if invalid:
                        sc_inv += 1
                    elif weak:
                        sc_weak += 1

                    if (not invalid) and not (REJECT_WEAK and weak) and 60 <= dist <= 5500:
                        sc_valid += 1
                        d_m = dist / 1000.0
                        ranges[ang] = d_m
                        new_batch[gui_ang] = (d_m, now)
                        scan_buf[ros_idx] = d_m
                    else:
                        ranges[ang] = float('inf')

        if new_batch:
            with lock:
                lidar_points.update(new_batch)
    sock.close()

# =========================================================
# YARDIMCI: SEKTÖR OKUMA (gui açısı: 0 = burun)
# =========================================================
def sector_vals(r_list, center_gui, spread):
    c = gui_to_raw(center_gui)
    vals = []
    for d in range(c - spread, c + spread + 1):
        dist = r_list[d % 360]
        if 0.05 < dist < 6.0:
            vals.append(dist)
    return vals

def valid_ratio(r_list):
    return sum(1 for x in r_list if 0.05 < x < 6.0) / 360.0

def rear_blocked(r_list):
    """Arkada engel (veya el) var mı? Geçersiz okumaları da hesaba katar."""
    c = gui_to_raw(180)
    total = 2 * REAR_SPREAD + 1
    valid = []
    for d in range(c - REAR_SPREAD, c + REAR_SPREAD + 1):
        dist = r_list[d % 360]
        if 0.05 < dist < 6.0:
            valid.append(dist)

    if valid and min(valid) < REAR_CLEAR_DIST:
        return True

    if REAR_INVALID_BLOCKS:
        invalid_ratio = 1.0 - (len(valid) / total)
        if invalid_ratio > 0.5 and valid_ratio(r_list) > 0.3:
            return True
    return False

def get_front_distances(r_list):
    center = sector_vals(r_list, 0, 12)
    wide = sector_vals(r_list, 0, 35)

    if center:
        d_center = min(center)
    else:
        d_center = 0.15 if valid_ratio(r_list) > 0.3 else 5.0

    d_wide = min(wide) if wide else d_center
    return d_center, d_wide

def find_best_direction(r_list):
    best_angle, best_score = 0, -1e9
    for a in range(-170, 180, 10):
        vals = sector_vals(r_list, a, 15)
        if not vals:
            score = 1.0
        else:
            avg = sum(vals) / len(vals)
            mn = min(vals)
            score = min(avg, 3.0) + 0.5 * min(mn, 1.0)
            if mn < 0.30:
                score -= 1.0
        score -= (abs(a) / 180.0) * 0.5
        if score > best_score:
            best_score, best_angle = score, a
    return 1 if best_angle < 0 else -1

# =========================================================
# LIDAR TABANLI SIKIŞMA (HAREKETSİZLİK) ANALİZİ
# =========================================================
def reset_stall_tracker(now, grace=STALL_GRACE):
    global prev_scan_snapshot, last_stall_check_time, stall_streak, stall_grace_until
    prev_scan_snapshot = None
    last_stall_check_time = now
    stall_streak = 0
    stall_grace_until = now + grace

def check_physical_motion(current_ranges, now):
    """Sıkışma onaylandığı an bir kez True döner."""
    global prev_scan_snapshot, last_stall_check_time, stall_streak, unstuck_count

    if current_left == 0 and current_right == 0:
        reset_stall_tracker(now)
        return False

    if now < stall_grace_until:
        return False

    if prev_scan_snapshot is None:
        prev_scan_snapshot = list(current_ranges)
        last_stall_check_time = now
        return False

    if now - last_stall_check_time < STALL_CHECK_INTERVAL:
        return False

    near_vals = [x for x in current_ranges if 0.05 < x < 6.0]
    near_min = min(near_vals) if near_vals else 99.0
    if near_min > STALL_NEAR_DIST:
        prev_scan_snapshot = list(current_ranges)
        last_stall_check_time = now
        stall_streak = 0
        unstuck_count = 0
        return False

    diffs = []
    for i in range(360):
        d1 = prev_scan_snapshot[i]
        d2 = current_ranges[i]
        if 0.10 < d1 < 4.0 and 0.10 < d2 < 4.0:
            diffs.append(abs(d1 - d2))

    prev_scan_snapshot = list(current_ranges)
    last_stall_check_time = now

    if len(diffs) < 30:
        stall_streak = 0
        return False

    diffs.sort(reverse=True)
    top = diffs[:max(1, len(diffs) // 4)]
    movement = sum(top) / len(top)

    if movement < STALL_THRESHOLD_M:
        stall_streak += 1
    else:
        stall_streak = 0
        unstuck_count = 0

    if stall_streak >= STALL_CONFIRM:
        stall_streak = 0
        return True
    return False

# =========================================================
# OTONOM KARAR VE KURTULMA MOTORU
# =========================================================
def set_state(new_state, now):
    global robot_state, turn_start_time
    robot_state = new_state
    turn_start_time = now
    reset_stall_tracker(now, grace=0.6)

def reactive_step():
    global turn_dir, unstuck_until_time, unstuck_mode, unstuck_count, stuck_display_until
    now = time.time()

    with lock:
        r_copy = list(ranges)
        scan_age = now - last_scan_time

    if scan_age > 0.5:
        stop()
        set_state('SURUS', now)
        return "No LiDAR signal -> Stopped"

    d_front, d_wide = get_front_distances(r_copy)
    blocked = d_front < FRONT_BLOCK_DIST or d_wide < WIDE_BLOCK_DIST

    if robot_state != 'KURTULMA':
        if check_physical_motion(r_copy, now):
            unstuck_count += 1
            stuck_display_until = now + 1.5
            turn_dir = find_best_direction(r_copy)

            if rear_blocked(r_copy) or unstuck_count >= 2:
                unstuck_mode = 'spin'
                unstuck_until_time = now + UNSTUCK_SPIN_TIME
                if unstuck_count >= 3:
                    turn_dir = -turn_dir
                spin(turn_dir, UNSTUCK_PWM)
            else:
                unstuck_mode = 'back'
                unstuck_until_time = now + UNSTUCK_BACK_TIME
                send_velocity(-UNSTUCK_PWM, -UNSTUCK_PWM)

            set_state('KURTULMA', now)
            return f"⚠️ STALL! Recovery ({unstuck_mode}) #{unstuck_count}"

    if robot_state == 'KURTULMA':
        if unstuck_mode == 'back' and rear_blocked(r_copy):
            unstuck_mode = 'spin'
            unstuck_until_time = now + UNSTUCK_SPIN_TIME
            spin(turn_dir, UNSTUCK_PWM)
            return "⚠️ Rear blocked! Reverse cancelled -> Spinning in place"

        if now < unstuck_until_time:
            if unstuck_mode == 'back':
                send_velocity(-UNSTUCK_PWM, -UNSTUCK_PWM)
                return "⚠️ Recovery: Backing up..."
            spin(turn_dir, UNSTUCK_PWM)
            return "⚠️ Recovery: Spinning in place..."

        turn_dir = find_best_direction(r_copy)
        set_state('DONUS', now)
        reset_stall_tracker(now, grace=1.2)
        return "Recovery done -> Heading to the most open space"

    if robot_state == 'DONUS':
        elapsed = now - turn_start_time

        if elapsed >= MIN_TURN_TIME and d_front >= TARGET_CLEAR_DIST and d_wide >= 0.40:
            set_state('SURUS', now)
            send_velocity(CRUISE_SPEED, CRUISE_SPEED)
            return f"Opening found (Front: {d_front:.2f}m) -> Driving straight"

        if elapsed > TURN_TIMEOUT:
            turn_dir = -turn_dir
            set_state('DONUS', now)
            return "Turn timeout -> Direction changed"

        spin(turn_dir, TURN_SPEED)
        yon = "LEFT" if turn_dir == 1 else "RIGHT"
        return f"Turning toward open corridor [{yon}] (Front: {d_front:.2f}m)"

    if blocked:
        turn_dir = find_best_direction(r_copy)
        set_state('DONUS', now)
        spin(turn_dir, TURN_SPEED)
        return f"Front blocked (C:{d_front:.2f} W:{d_wide:.2f}) -> Heading to open space"

    send_velocity(CRUISE_SPEED, CRUISE_SPEED)
    return f"Path clear (Front: {d_front:.2f}m) -> Driving straight"

# =========================================================
# HEDEFE GİDİŞ (haritada tıkla -> güvenli yol planla -> sür)
# =========================================================
nav_lock = threading.Lock()
nav = {
    "active": False, "state": "IDLE",   # IDLE, PLAN, FOLLOW, AVOID, RECOVER
    "goal": None, "goal_eff": None, "path": [], "idx": 0, "plan": None,
    "t0": 0.0, "replans": 0, "prog_pose": None, "prog_time": 0.0,
    "rec_until": 0.0, "rec_mode": "back", "avoid_until": 0.0, "avoid_dir": 1,
    "boost": 0,      # Öğrenilen ek PWM (ölü bölgeyi aşmak için); oturum boyunca korunur
    "spin_t0": None, "lowq_t0": None,
}

def wrap_pi(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def _dist_from(mask, kmax):
    """Engellere olan yaklaşık mesafe (hücre). Çapraz/artı genişletmeyi dönüşümlü yapar (sekizgen ~ daire)."""
    dist = np.full(mask.shape, kmax + 1, dtype=np.int16)
    dist[mask] = 0
    cur = mask.copy()
    for k in range(1, kmax + 1):
        nb = cur.copy()
        nb[1:, :] |= cur[:-1, :]; nb[:-1, :] |= cur[1:, :]
        nb[:, 1:] |= cur[:, :-1]; nb[:, :-1] |= cur[:, 1:]
        if k % 2 == 0:
            nb[1:, 1:] |= cur[:-1, :-1]; nb[1:, :-1] |= cur[:-1, 1:]
            nb[:-1, 1:] |= cur[1:, :-1]; nb[:-1, :-1] |= cur[1:, 1:]
        dist[nb & ~cur] = k
        cur = nb
    return dist

def _los(bl, a, b):
    x0, y0 = a; x1, y1 = b
    n = max(abs(x1 - x0), abs(y1 - y0))
    for i in range(n + 1):
        t = i / n if n else 0.0
        x = int(round(x0 + (x1 - x0) * t)); y = int(round(y0 + (y1 - y0) * t))
        if bl[y][x]:
            return False
    return True

def _astar(blocked, penalty, start, goal, max_exp=400000):
    H, W = blocked.shape
    sx, sy = start; gx, gy = goal
    if start == goal:
        return [start]
    bl = blocked.tolist()
    pen = penalty.tolist()
    SQ2 = 1.41421356
    nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]

    def hfun(x, y):
        dx, dy = abs(x - gx), abs(y - gy)
        return (dx + dy) + (SQ2 - 2.0) * min(dx, dy)

    g = {(sx, sy): 0.0}
    came = {}
    openh = [(hfun(sx, sy), 0.0, sx, sy)]
    closed = set()
    exp = 0
    while openh:
        _, gc, x, y = heapq.heappop(openh)
        if (x, y) in closed:
            continue
        closed.add((x, y))
        if x == gx and y == gy:
            path = [(x, y)]
            while (x, y) in came:
                x, y = came[(x, y)]
                path.append((x, y))
            path.reverse()
            return path
        exp += 1
        if exp > max_exp:
            return None
        for dx, dy, c in nbrs:
            nx, ny = x + dx, y + dy
            if nx < 0 or ny < 0 or nx >= W or ny >= H or bl[ny][nx]:
                continue
            if dx and dy and (bl[y][nx] or bl[ny][x]):   # köşeden kesme yok
                continue
            ng = gc + c * (1.0 + pen[ny][nx])
            if ng < g.get((nx, ny), 1e18):
                g[(nx, ny)] = ng
                came[(nx, ny)] = (x, y)
                heapq.heappush(openh, (ng + hfun(nx, ny), ng, nx, ny))
    return None

def live_scan_points(r_list, pose):
    """Canlı LiDAR noktalarını harita çerçevesine çevirir (haritada olmayan yeni engeller için)."""
    px, py, yaw = pose
    pts = []
    c, s = math.cos(yaw), math.sin(yaw)
    for ang in range(360):
        d = r_list[ang]
        if 0.10 < d < 2.5:
            g = math.radians(raw_to_gui(ang))   # saat yönünde, burundan
            xf = d * math.cos(g)
            yl = -d * math.sin(g)
            pts.append((px + xf * c - yl * s, py + xf * s + yl * c))
    return pts

def plan_path(grid, start_xy, goal_xy, extra_pts):
    """Döner: (yol[(x,y)...] veya None, hata_mesajı)"""
    arr, w, h, res, ox, oy = grid
    f = max(1, int(round(NAV_PLAN_RES / res)))
    cs = res * f
    hh, ww = h // f, w // f
    if hh < 3 or ww < 3:
        return None, "Map too small"
    a = arr[:hh * f, :ww * f]
    occ = a >= 50
    for (x, y) in extra_pts:
        ix = int((x - ox) / res); iy = int((y - oy) / res)
        if 0 <= ix < ww * f and 0 <= iy < hh * f:
            occ[iy, ix] = True
    unk = a < 0
    occ_b = occ.reshape(hh, f, ww, f).any(axis=(1, 3))
    unk_b = unk.reshape(hh, f, ww, f).mean(axis=(1, 3)) > 0.2

    r_hard = max(1, int(math.ceil((ROBOT_RADIUS_M + SAFETY_MARGIN_M) / cs)))
    r_soft = r_hard + max(1, int(math.ceil(NAV_SOFT_ZONE_M / cs)))
    dist = _dist_from(occ_b, r_soft)
    blocked = (dist <= r_hard) | unk_b
    penalty = np.where(dist <= r_soft,
                       (r_soft - dist) / float(r_soft - r_hard) * NAV_SOFT_COST, 0.0)

    def to_cell(x, y):
        return int((x - ox) / cs), int((y - oy) / cs)

    sx, sy = to_cell(*start_xy)
    if not (0 <= sx < ww and 0 <= sy < hh):
        return None, "Robot appears to be outside the map"
    # Duvara çok yakın başlansa bile çıkabilsin: başlangıç çevresindeki şişirilmiş bölgeyi aç
    ys = slice(max(0, sy - r_hard), min(hh, sy + r_hard + 1))
    xs = slice(max(0, sx - r_hard), min(ww, sx + r_hard + 1))
    blocked[ys, xs] = occ_b[ys, xs]

    gx, gy = to_cell(*goal_xy)
    if not (0 <= gx < ww and 0 <= gy < hh):
        return None, "Goal is outside the map"
    goal_world = goal_xy
    if blocked[gy, gx]:
        R = int(math.ceil(0.5 / cs))
        y0_, y1_ = max(0, gy - R), min(hh, gy + R + 1)
        x0_, x1_ = max(0, gx - R), min(ww, gx + R + 1)
        yy, xx = np.nonzero(~blocked[y0_:y1_, x0_:x1_])
        if len(yy) == 0:
            return None, "Goal is unsafe or in an unknown area. Pick a white (free) spot on the map."
        yy = yy + y0_; xx = xx + x0_
        k = int(np.argmin((yy - gy) ** 2 + (xx - gx) ** 2))
        gy, gx = int(yy[k]), int(xx[k])
        goal_world = (ox + (gx + 0.5) * cs, oy + (gy + 0.5) * cs)

    cells = _astar(blocked, penalty, (sx, sy), (gx, gy))
    if cells is None:
        return None, "No safe path to the goal found"

    # Görüş hattıyla sadeleştir
    bl = blocked.tolist()
    pruned = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _los(bl, cells[i], cells[j]):
            j -= 1
        pruned.append(cells[j])
        i = j

    pts = [tuple(start_xy)]
    for (cx, cy) in pruned[1:-1]:
        pts.append((ox + (cx + 0.5) * cs, oy + (cy + 0.5) * cs))
    pts.append(goal_world)

    # 12 cm aralıklı sık nokta dizisi
    dense = [pts[0]]
    for (x1, y1), (x2, y2) in zip(pts[:-1], pts[1:]):
        seg = math.hypot(x2 - x1, y2 - y1)
        n = max(1, int(seg / 0.12))
        for k in range(1, n + 1):
            t = k / n
            dense.append((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t))
    return dense, ""

def _plan_worker(goal, pose, grid, r_copy):
    try:
        extra = live_scan_points(r_copy, pose)
        result = plan_path(grid, (pose[0], pose[1]), goal, extra)
    except Exception as e:
        result = (None, f"Planning error: {e}")
    with nav_lock:
        nav["plan"] = result

def start_plan(goal, pose):
    grid = ros_bridge.get_grid() if ros_bridge else None
    if grid is None:
        return False
    with lock:
        r_copy = list(ranges)
    with nav_lock:
        nav["plan"] = None
    nav["state"] = "PLAN"
    threading.Thread(target=_plan_worker, args=(goal, pose, grid, r_copy), daemon=True).start()
    return True

def _nav_ui(mode_text, color, info=None):
    try:
        mode_label.config(text=mode_text, fg=color)
        if info is not None:
            map_info.config(text=info)
    except Exception:
        pass

def nav_cancel(msg="Goal cancelled"):
    nav.update(active=False, state="IDLE", path=[], goal=None, goal_eff=None)
    stop_hard()
    nav["msg"] = msg
    if not is_autonomous:
        _nav_ui("MODE: MANUAL", "#2980b9", msg)
    else:
        _nav_ui("MODE: AUTONOMOUS", "#27ae60", msg)

def nav_set_goal(wx, wy):
    if not ROS_OK or ros_bridge is None:
        return False, "ROS / Cartographer not available"
    if not mapping_running():
        return False, "Press 'Start Mapping' first (Cartographer is required for localization)"
    pose = ros_bridge.get_pose()
    if pose is None:
        return False, "No robot pose (TF). Is Cartographer running and receiving scans?"
    if ros_bridge.get_grid() is None:
        return False, "Map not built yet"
    if calib["active"]:
        return False, "Direction test in progress, please wait"
    if is_autonomous:
        toggle_mode()
    pressed_keys.clear()
    now = time.time()
    nav.update(active=True, goal=(wx, wy), goal_eff=(wx, wy), path=[], idx=0,
               t0=now, replans=0, prog_pose=pose, prog_time=now)
    start_plan((wx, wy), pose)
    _nav_ui("MODE: GO TO GOAL", "#e67e22", "Planning path... (cancel: SPACE / ESC)")
    return True, "Planning path..."

def nav_drive(l, r):
    """Motor kalkış eşiği telafisi: hareket etmesi gereken her tekerleğe yeterli PWM ver."""
    l, r = float(l), float(r)
    m = max(abs(l), abs(r))
    if m < 1:
        send_velocity(0, 0)
        return

    floor = min(255, NAV_MIN_PWM + nav["boost"])
    # İki komut da çok düşükse önce en büyük komutu kalkış eşiğine çıkar.
    if m < floor:
        k = floor / m
        l *= k
        r *= k

    # Hızlı tarafı değiştirmeden, kalkış eşiğinin altında kalan tarafı yükselt.
    # Böylece bir tarafın neredeyse durup aracın aniden yön değiştirmesi azaltılır.
    if abs(l) > 0:
        l = math.copysign(max(abs(l), floor), l)
    if abs(r) > 0:
        r = math.copysign(max(abs(r), floor), r)
    send_velocity(round(l), round(r))

def nav_spin(direction):
    # Fiziksel dönüş yönü kodun varsayımının tersiyse NAV_TURN_SIGN=-1 düzeltir.
    spin(NAV_TURN_SIGN * direction, min(255, NAV_SPIN_PWM + nav["boost"]))

def nav_replan(pose, why):
    nav["replans"] += 1
    if nav["replans"] > NAV_MAX_REPLANS:
        msg = "Could not reach the goal (too many obstacles/replans)"
        nav_cancel(msg)
        return msg
    stop()
    start_plan(nav["goal"], pose)
    return f"{why} -> replanning ({nav['replans']}/{NAV_MAX_REPLANS})"

def nav_step():
    """Her 50 ms'de bir çağrılır. Durum metni döner."""
    if not nav["active"]:
        return ""
    now = time.time()
    if now - nav["t0"] > NAV_TIMEOUT_S:
        nav_cancel("Timeout, goal cancelled")
        return nav["msg"]

    with lock:
        r_copy = list(ranges)
        scan_age = now - last_scan_time
    if scan_age > 0.5:
        # Hedefe gidişi aktif halde sonsuza kadar STOP komutunda bırakma.
        # Navigasyonu kapat ki operatör manuel kontrolü hemen geri alabilsin.
        nav_cancel("⚠️ LiDAR data lost: navigation cancelled; manual control released")
        return nav["msg"]
    pose = ros_bridge.get_pose() if ros_bridge else None
    if pose is None:
        # TF kaybolunca da aktif hedef tekrar tekrar stop üretmesin.
        nav_cancel("⚠️ Robot pose (TF) lost: navigation cancelled")
        return nav["msg"]
    x, y, yaw = pose

    # LiDAR paketleri kayıpsa konum güvenilmez: dur ve bekle
    pk = scan_stats["pkts"]
    if nav["state"] == "FOLLOW" and 0 < pk < NAV_MIN_PKTS:
        if nav["lowq_t0"] is None:
            nav["lowq_t0"] = now
        if now - nav["lowq_t0"] > 8.0:
            nav["lowq_t0"] = None
            nav_cancel("⚠️ LiDAR packet loss lasted too long: navigation cancelled")
            return nav["msg"]
        stop()
        nav["prog_time"] = now
        return f"⚠️ LiDAR packet loss ({pk}/90): pose unreliable, waiting"
    nav["lowq_t0"] = None

    # ---- Planlama ----
    if nav["state"] == "PLAN":
        stop()
        with nav_lock:
            res = nav["plan"]
        if res is None:
            return "Planning path..."
        with nav_lock:
            nav["plan"] = None
        path, err = res
        if path is None:
            nav_cancel(err)
            return err
        nav["path"] = path
        nav["goal_eff"] = path[-1]
        nav["idx"] = 0
        nav["state"] = "FOLLOW"
        nav["prog_pose"] = pose
        nav["prog_time"] = now

    # ---- Kurtulma (sıkışma) ----
    if nav["state"] == "RECOVER":
        if nav["rec_mode"] == "back" and rear_blocked(r_copy):
            nav["rec_mode"] = "spin"
            nav["rec_until"] = now + UNSTUCK_SPIN_TIME
        if now < nav["rec_until"]:
            if nav["rec_mode"] == "back":
                send_velocity(-UNSTUCK_PWM, -UNSTUCK_PWM)
                return "⚠️ Stuck: backing up"
            spin(nav["avoid_dir"], UNSTUCK_PWM)
            return "⚠️ Stuck: spinning in place"
        return nav_replan(pose, "After stall")

    # ---- Engel kaçınma (dönerek açık yön bul) ----
    if nav["state"] == "AVOID":
        d_front, d_wide = get_front_distances(r_copy)
        if (d_front >= 0.5 and d_wide >= 0.3) or now > nav["avoid_until"]:
            return nav_replan(pose, "Obstacle avoidance done")
        nav_spin(nav["avoid_dir"])
        return f"⚠️ Obstacle: turning toward open side (front {d_front:.2f} m)"

    # ---- Yolu takip ----
    gx, gy = nav["goal_eff"]
    dist_goal = math.hypot(gx - x, gy - y)
    if dist_goal < NAV_GOAL_TOL_M:
        nav_cancel("✅ Goal reached")
        return nav["msg"]

    path = nav["path"]
    i0 = nav["idx"]
    best, bd = i0, 1e9
    for i in range(i0, min(i0 + 15, len(path))):
        d = math.hypot(path[i][0] - x, path[i][1] - y)
        if d < bd:
            best, bd = i, d
    nav["idx"] = best
    if bd > 0.45:
        return nav_replan(pose, "Off path")

    ti = min(best + max(1, int(NAV_LOOKAHEAD_M / 0.12)), len(path) - 1)
    tx, ty = path[ti]
    err = wrap_pi(math.atan2(ty - y, tx - x) - yaw)    # >0: sola dön

    # Sıkışma: komut var ama konum/yön değişmiyor
    pp = nav["prog_pose"]
    moved = math.hypot(x - pp[0], y - pp[1]) > 0.05 or abs(wrap_pi(yaw - pp[2])) > 0.2
    if moved:
        nav["prog_pose"] = pose
        nav["prog_time"] = now
    elif (current_left or current_right) and now - nav["prog_time"] > NAV_BOOST_S \
            and nav["boost"] < NAV_MAX_BOOST:
        # Muhtemelen motor ölü bölgesi: önce gücü artır, sonra sıkışma say
        nav["boost"] = min(NAV_MAX_BOOST, nav["boost"] + NAV_BOOST_STEP)
        nav["prog_time"] = now
    elif now - nav["prog_time"] > NAV_STUCK_S and (current_left or current_right):
        nav["state"] = "RECOVER"
        nav["avoid_dir"] = find_best_direction(r_copy)
        if rear_blocked(r_copy):
            nav["rec_mode"] = "spin"; nav["rec_until"] = now + UNSTUCK_SPIN_TIME
        else:
            nav["rec_mode"] = "back"; nav["rec_until"] = now + UNSTUCK_BACK_TIME
        return "⚠️ Stall detected"

    # Önce hedef yöne dön
    if abs(err) > NAV_SPIN_ERR:
        if nav["spin_t0"] is None:
            nav["spin_t0"] = now
        cyc = NAV_SPIN_ON_S + NAV_SPIN_OFF_S
        if NAV_SPIN_OFF_S > 0 and (now - nav["spin_t0"]) % cyc >= NAV_SPIN_ON_S:
            stop()      # kısa duraklama: Cartographer konumu oturtsun
            return f"Turning toward goal (pause) ({math.degrees(err):+.0f}°) | remaining {dist_goal:.2f} m"
        nav_spin(1 if err > 0 else -1)
        return f"Turning toward goal ({math.degrees(err):+.0f}°) | remaining {dist_goal:.2f} m"
    nav["spin_t0"] = None

    # İleri sürüş: canlı LiDAR ile güvenlik
    d_front, d_wide = get_front_distances(r_copy)
    if d_front < FRONT_BLOCK_DIST or d_wide < WIDE_BLOCK_DIST:
        stop()
        nav["state"] = "AVOID"
        nav["avoid_dir"] = find_best_direction(r_copy)
        nav["avoid_until"] = now + 3.0
        return f"⚠️ Obstacle! (front {d_front:.2f} m) -> stopped"

    v = float(NAV_SPEED)
    if d_front < 0.7:
        v *= 0.85
    if dist_goal < 0.4:
        v *= 0.85
    v *= max(0.7, 1.0 - abs(err) / 1.5)

    steer = NAV_K_STEER * err            # + : fiziksel SOLA dön
    ls = min(sector_vals(r_copy, -45, 25) or [9.0])    # sol yan
    rs = min(sector_vals(r_copy, 45, 25) or [9.0])     # sağ yan
    if ls < NAV_SIDE_CLEAR:
        steer -= NAV_K_REPEL * (NAV_SIDE_CLEAR - ls)
    if rs < NAV_SIDE_CLEAR:
        steer += NAV_K_REPEL * (NAV_SIDE_CLEAR - rs)
    steer = NAV_TURN_SIGN * steer        # motor yönü ters bağlıysa komut işaretini çevir
    steer = max(-0.7 * v, min(0.7 * v, steer))
    nav_drive(v - steer, v + steer)
    return (f"Driving to goal | remaining {dist_goal:.2f} m | front {d_front:.2f} m"
            f" | PWM {current_left}/{current_right} (+{nav['boost']})")

# =========================================================
# YÖN TESTİ (LiDAR yönü / motor yönü / ileri eksen kalibrasyonu)
# =========================================================
CAL_SPIN_S = 1.3
CAL_PAUSE_S = 0.7
CAL_FWD_S = 1.0
calib = {"active": False, "phase": "", "t0": 0.0, "yaw_prev": 0.0, "acc": 0.0, "p0": None, "msg": ""}

def restart_mapping():
    stop_mapping()
    ok, msg = start_mapping()
    try:
        if ok:
            map_btn.config(text="Stop Mapping", bg="#e74c3c", activebackground="#c0392b")
    except Exception:
        pass
    trail.clear()
    return ok, msg

def calib_finish(msg):
    calib.update(active=False, phase="", msg=msg)
    stop_hard()
    _nav_ui("MODE: MANUAL", "#2980b9", msg)

def calib_start():
    if not ROS_OK or ros_bridge is None:
        return False, "ROS / Cartographer not available"
    if not mapping_running():
        return False, "Press 'Start Mapping' first"
    pose = ros_bridge.get_pose()
    if pose is None:
        return False, "No robot pose (TF)"
    if is_autonomous or nav["active"]:
        return False, "Stop autonomous/goal mode first"
    pressed_keys.clear()
    calib.update(active=True, phase="SPIN", t0=time.time(), yaw_prev=pose[2], acc=0.0, p0=None)
    _nav_ui("MODE: DIRECTION TEST", "#8e44ad",
            "Direction test: the robot will first turn left, then drive forward a little (cancel: ESC/SPACE)")
    return True, "Direction test started"

def calib_step():
    global LIDAR_ANGLES_CLOCKWISE, FRONT_RAW, NAV_TURN_SIGN
    now = time.time()
    with lock:
        r_copy = list(ranges)
        scan_age = now - last_scan_time
    pose = ros_bridge.get_pose() if ros_bridge else None
    t = now - calib["t0"]
    if pose is None or scan_age > 0.5:
        stop()
        if t > 8.0:
            calib_finish("Direction test cancelled: no pose or LiDAR data")
            return calib["msg"]
        return "Direction test: waiting for pose/LiDAR"
    x, y, yaw = pose
    ph = calib["phase"]

    if ph == "SPIN":
        calib["acc"] += wrap_pi(yaw - calib["yaw_prev"])
        calib["yaw_prev"] = yaw
        if t < CAL_SPIN_S:
            spin(1, min(255, NAV_SPIN_PWM + nav["boost"]))      # HAM 'sol' komutu (işaret düzeltmesi yok)
            return f"Direction test 1/2: turning left (on map {math.degrees(calib['acc']):+.0f}°)"
        stop_hard()
        acc = calib["acc"]
        if abs(acc) < 0.15:
            calib_finish("Direction test: the robot did not turn or the pose did not change. Increase NAV_SPIN_PWM and try again.")
            return calib["msg"]
        turned_left = messagebox.askyesno(
            "Direction Test",
            "Did the robot PHYSICALLY turn LEFT?\n(Looking at the robot from above; Yes = left, No = right)",
            parent=root)
        phys = 1 if turned_left else -1
        yaw_sign = 1 if acc > 0 else -1
        flip = (phys != yaw_sign)          # ROS yönü fiziksel yönle uyuşmuyorsa LiDAR yönü ters
        if flip:
            LIDAR_ANGLES_CLOCKWISE = not LIDAR_ANGLES_CLOCKWISE
        NAV_TURN_SIGN = phys               # 'sol' komutu fiziksel sağa dönüyorsa motor yönü ters
        save_calib()
        notes = []
        if flip:
            notes.append(f"LiDAR direction corrected (LIDAR_ANGLES_CLOCKWISE={LIDAR_ANGLES_CLOCKWISE})")
        if phys == -1:
            notes.append("'left' command turns the robot right: NAV_TURN_SIGN=-1 (check the left/right motor wiring on the ESP32)")
        if flip:
            restart_mapping()
            calib_finish(" | ".join(notes) + ". Map reset; wait a few seconds and run the test again.")
            return calib["msg"]
        if notes:
            map_info.config(text=" | ".join(notes))
        calib.update(phase="PAUSE1", t0=time.time())
        return "Direction test 1/2 done: turn direction is correct"

    if ph == "PAUSE1":
        stop()
        if t < CAL_PAUSE_S:
            return "Direction test: pose settling..."
        d_front, _ = get_front_distances(r_copy)
        if d_front < 0.6:
            calib_finish("✅ Turn direction is correct. The forward-axis test needs ~1 m of free space ahead; move the robot to an open area and try again.")
            return calib["msg"]
        calib.update(phase="FWD", t0=now, p0=(x, y, yaw))
        return "Direction test 2/2: driving forward"

    if ph == "FWD":
        d_front, d_wide = get_front_distances(r_copy)
        if t < CAL_FWD_S and d_front >= 0.45 and d_wide >= 0.25:
            nav_drive(NAV_SPEED, NAV_SPEED)
            return "Direction test 2/2: driving forward"
        stop_hard()
        calib.update(phase="PAUSE2", t0=now)
        return "Direction test: measuring..."

    if ph == "PAUSE2":
        stop()
        if t < CAL_PAUSE_S:
            return "Direction test: measuring..."
        x0, y0, yaw0 = calib["p0"]
        dist = math.hypot(x - x0, y - y0)
        if dist < 0.10:
            calib_finish(f"Direction test: the robot did not move far enough ({dist * 100:.0f} cm), forward axis could not be measured.")
            return calib["msg"]
        phi = math.degrees(wrap_pi(math.atan2(y - y0, x - x0) - yaw0))
        if abs(phi) < 20:
            calib_finish(f"✅ Direction test passed: turn and forward axis are correct (deviation {phi:+.0f}°)")
            return calib["msg"]
        FRONT_RAW = int(round(FRONT_RAW - phi if LIDAR_ANGLES_CLOCKWISE else FRONT_RAW + phi)) % 360
        save_calib()
        restart_mapping()
        calib_finish(f"Forward axis was off by {phi:+.0f}° -> FRONT_RAW set to {FRONT_RAW}. "
                     "Map reset; wait a few seconds and run the test again.")
        return calib["msg"]

    calib_finish("Direction test: unknown state, cancelled")
    return calib["msg"]

# =========================================================
# ARAYÜZ
# =========================================================
root = tk.Tk()
root.title("ESP32 Rover - Manual & Autonomous & LiDAR & Cartographer")
root.geometry("1200x720")

left_panel = tk.Frame(root, padx=15, pady=15)
left_panel.pack(side="left", fill="y")

ttk.Separator(root, orient="vertical").pack(side="left", fill="y", padx=5)

right_panel = tk.Frame(root, padx=10, pady=10)
right_panel.pack(side="right", fill="both", expand=True)

# ---------------- Sol Kontroller ----------------
tk.Label(left_panel, text="ESP32 ROVER", font=("Arial", 18, "bold")).pack(pady=(0, 6))

mode_label = tk.Label(left_panel, text="MODE: MANUAL", font=("Arial", 12, "bold"), fg="#2980b9")
mode_label.pack(pady=(0, 6))

dpad_buttons = []

def set_dpad_state(enabled):
    for b in dpad_buttons:
        b.config(state=("normal" if enabled else "disabled"))

def toggle_mode():
    global is_autonomous, unstuck_count
    if nav["active"]:
        nav_cancel("Navigation cancelled (SPACE)")
        return
    if calib["active"]:
        calib_finish("Direction test cancelled (SPACE)")
        return
    is_autonomous = not is_autonomous
    unstuck_count = 0
    pressed_keys.clear()
    stop_hard()
    set_state('SURUS', time.time())

    if is_autonomous:
        toggle_btn.config(text="STOP AUTONOMOUS [SPACE]", bg="#e74c3c", activebackground="#c0392b")
        mode_label.config(text="MODE: AUTONOMOUS", fg="#27ae60")
        set_dpad_state(False)
        scale.state(["disabled"])
    else:
        toggle_btn.config(text="START AUTONOMOUS [SPACE]", bg="#2ecc71", activebackground="#27ae60")
        mode_label.config(text="MODE: MANUAL", fg="#2980b9")
        set_dpad_state(True)
        scale.state(["!disabled"])
        status_label.config(text="Autonomous stopped, manual control active")

toggle_btn = tk.Button(
    left_panel, text="START AUTONOMOUS [SPACE]", command=toggle_mode,
    font=("Arial", 11, "bold"), bg="#2ecc71", fg="#ffffff", activeforeground="#ffffff",
    height=2, relief="flat", takefocus=0,
)
toggle_btn.pack(fill="x", pady=(0, 10))

speed_label = tk.Label(left_panel, text="Speed: 100", font=("Arial", 12))
speed_label.pack()

scale = ttk.Scale(left_panel, from_=20, to=255, orient="horizontal", command=speed_changed, length=240)
scale.set(100)
scale.pack(pady=10)

btn_frame = tk.Frame(left_panel)
btn_frame.pack(pady=10)

def btn(t, c, r, col):
    b = tk.Button(btn_frame, text=t, command=c, width=6, height=2,
                  font=("Arial", 11, "bold"), takefocus=0)
    b.grid(row=r, column=col, padx=4, pady=4)
    dpad_buttons.append(b)

btn("↑", forward, 0, 1)
btn("←", left, 1, 0)
btn("STOP", manual_stop, 1, 1)
btn("→", right, 1, 2)
btn("↓", backward, 2, 1)

motor_label = tk.Label(left_panel, text="Left: 0   Right: 0", font=("Arial", 13))
motor_label.pack(pady=8)

link_label = tk.Label(left_panel, text="Link: waiting", font=("Arial", 10, "bold"), fg="#7f8c8d")
link_label.pack(pady=4)

stall_indicator = tk.Label(left_panel, text="Motion status: Normal", font=("Arial", 10, "bold"), fg="#27ae60")
stall_indicator.pack(pady=4)

status_label = tk.Label(left_panel, text="Waiting for ESP32...", font=("Arial", 10),
                        fg="#7f8c8d", wraplength=250, justify="left")
status_label.pack(pady=5)

info_txt = (
    "• Manual: W/A/S/D or arrow keys\n"
    "• SPACE: Toggle autonomous mode\n"
    "• Start Cartographer from the Map tab\n"
    "  and drive the robot around to build a map.\n"
    "• Turning slowly gives better mapping."
)
tk.Label(left_panel, text=info_txt, font=("Arial", 9), fg="#7f8c8d", justify="left").pack(side="bottom", pady=10)

# ---------------- Sağ: Sekmeler ----------------
notebook = ttk.Notebook(right_panel)
notebook.pack(fill="both", expand=True)

radar_tab = tk.Frame(notebook)
map_tab = tk.Frame(notebook)
notebook.add(radar_tab, text="  Live LiDAR  ")
notebook.add(map_tab, text="  Map (Cartographer)  ")

# --- Radar sekmesi ---
lidar_stats = tk.Label(radar_tab, text="LiDAR starting...", font=("Arial", 11, "bold"))
lidar_stats.pack(anchor="w", pady=(4, 0))

weak_var = tk.BooleanVar(value=REJECT_WEAK)

def weak_changed():
    global REJECT_WEAK
    REJECT_WEAK = weak_var.get()

tk.Checkbutton(radar_tab, text="Reject weak-signal points", variable=weak_var,
               command=weak_changed, takefocus=0).pack(anchor="w")
diag_label = tk.Label(radar_tab, text="", font=("Arial", 9), fg="#7f8c8d")
diag_label.pack(anchor="w")

canvas = tk.Canvas(radar_tab, background="#101820", highlightthickness=0)
canvas.pack(fill="both", expand=True, pady=8)

# --- Harita sekmesi ---
map_bar = tk.Frame(map_tab)
map_bar.pack(fill="x", pady=(4, 0))

def toggle_mapping():
    if mapping_running():
        stop_mapping()
        map_btn.config(text="Start Mapping", bg="#2ecc71", activebackground="#27ae60")
        map_info.config(text="Cartographer stopped")
    else:
        ok, msg = start_mapping()
        if ok:
            map_btn.config(text="Stop Mapping", bg="#e74c3c", activebackground="#c0392b")
        map_info.config(text=msg)

def save_map():
    if not ROS_OK or ros_bridge is None or ros_bridge.get_map() is None:
        map_info.config(text="No map to save")
        return
    base = os.path.join(os.path.expanduser("~"), time.strftime("rover_map_%Y%m%d_%H%M%S"))
    try:
        os.makedirs(CFG_DIR, exist_ok=True)
        log = open(os.path.join(CFG_DIR, "map_saver.log"), "w")
        subprocess.Popen(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base],
                         stdout=log, stderr=subprocess.STDOUT)
        map_info.config(text=f"Saving: {base}.pgm/.yaml (requires nav2_map_server)")
    except FileNotFoundError:
        map_info.config(text="'ros2' not found")

def clear_trail():
    trail.clear()

map_btn = tk.Button(map_bar, text="Start Mapping", command=toggle_mapping,
                    font=("Arial", 10, "bold"), bg="#2ecc71", fg="#ffffff",
                    activeforeground="#ffffff", relief="flat", padx=10, takefocus=0)
map_btn.pack(side="left", padx=(0, 6))
tk.Button(map_bar, text="Save Map", command=save_map,
          font=("Arial", 10), padx=8, takefocus=0).pack(side="left", padx=3)
tk.Button(map_bar, text="Clear Trail", command=clear_trail,
          font=("Arial", 10), padx=8, takefocus=0).pack(side="left", padx=3)
tk.Button(map_bar, text="Direction Test", command=lambda: map_info.config(text=calib_start()[1]),
          font=("Arial", 10), padx=8, takefocus=0).pack(side="left", padx=3)
tk.Button(map_bar, text="Cancel Goal", command=lambda: nav_cancel("Goal cancelled"),
          font=("Arial", 10, "bold"), bg="#e67e22", fg="#ffffff", activeforeground="#ffffff",
          relief="flat", padx=8, takefocus=0).pack(side="left", padx=3)

map_info = tk.Label(map_bar, text="", font=("Arial", 10), fg="#7f8c8d")
map_info.pack(side="left", padx=10)

map_canvas = tk.Canvas(map_tab, background="#101820", highlightthickness=0)
map_canvas.pack(fill="both", expand=True, pady=8)
map_cache = {"key": None, "photo": None}
map_geom = {}

def on_map_click(event):
    g = map_geom
    if not g:
        return
    col = (event.x - g["x0"]) / g["eff"]
    row = (event.y - g["y0"]) / g["eff"]
    if not (0 <= col < g["w"] and 0 <= row < g["h"]):
        return
    wx = g["ox"] + col * g["res"]
    wy = g["oy"] + (g["h"] - row) * g["res"]
    ok, msg = nav_set_goal(wx, wy)
    map_info.config(text=msg)

map_canvas.bind("<Button-1>", on_map_click)
map_canvas.bind("<Button-3>", lambda e: nav_cancel("Goal cancelled"))

if not ROS_OK:
    map_btn.config(state="disabled")
    map_info.config(text="ROS 2 not found")

# =========================================================
# ÇİZİM FONKSİYONLARI
# =========================================================
def draw_radar():
    canvas.delete("all")
    w = max(canvas.winfo_width(), 200)
    h = max(canvas.winfo_height(), 200)
    cx, cy = w / 2, h / 2
    rad = max(20, min(w, h) * 0.44)
    scale_px = rad / MAX_RANGE_M

    for r_m in range(1, 7):
        r_p = r_m * scale_px
        canvas.create_oval(cx - r_p, cy - r_p, cx + r_p, cy + r_p, outline="#2c3e50")
        canvas.create_text(cx + 4, cy - r_p + 10, text=f"{r_m}m", fill="#7f8c8d",
                           anchor="w", font=("Arial", 8))

    canvas.create_line(cx - rad, cy, cx + rad, cy, fill="#2c3e50")
    canvas.create_line(cx, cy - rad, cx, cy + rad, fill="#2c3e50")
    canvas.create_line(cx, cy, cx, cy - (rad * 0.25), fill="#e74c3c", width=3, arrow=tk.LAST)
    canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#e74c3c", outline="")

    now = time.time()
    with lock:
        dead = [k for k, v in lidar_points.items() if now - v[1] > POINT_LIFETIME]
        for k in dead:
            del lidar_points[k]
        snapshot = dict(lidar_points)

    for deg, (dist, _) in snapshot.items():
        angle_rad = math.radians(deg)
        x = cx + math.sin(angle_rad) * dist * scale_px
        y = cy - math.cos(angle_rad) * dist * scale_px
        canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#2ecc71", outline="")

    lidar_stats.config(text=f"Active points: {len(snapshot)} / 360")
    st = scan_stats
    diag_label.config(text=f"Last rev -> packets: {st['pkts']}/90  valid points: {st['valid']}/360  "
                           f"invalid: {st['invalid']}  weak signal: {st['weak']}  bad packets: {st['badchk']}")

def update_trail():
    if not ROS_OK or ros_bridge is None:
        return
    pose = ros_bridge.get_pose()
    if pose is None:
        return
    x, y, _ = pose
    if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.03:
        trail.append((x, y))
        if len(trail) > 3000:
            del trail[:500]

def draw_map():
    c = map_canvas
    c.delete("all")
    cw = max(c.winfo_width(), 200)
    ch = max(c.winfo_height(), 200)

    if not ROS_OK:
        c.create_text(cw / 2, ch / 2, fill="#7f8c8d", font=("Arial", 12), justify="center",
                      text="ROS 2 (rclpy) not found.\nSource your ROS environment and restart the script.\n\n"
                           f"{ROS_ERR}")
        return

    # Cartographer beklenmedik şekilde kapandıysa uyar
    if carto_procs and not mapping_running():
        map_btn.config(text="Start Mapping", bg="#2ecc71", activebackground="#27ae60")
        map_info.config(text=f"Cartographer exited, logs: {CFG_DIR}")
        stop_mapping()

    m = ros_bridge.get_map() if ros_bridge else None
    if m is None:
        c.create_text(cw / 2, ch / 2, fill="#7f8c8d", font=("Arial", 12), justify="center",
                      text="Waiting for map.\nPress 'Start Mapping' and drive the robot around.")
        return

    ver, w, h, res, ox, oy, img = m
    s = min((cw - 10) / w, (ch - 10) / h)

    if PIL_OK:
        eff = s
        key = (ver, round(s, 3))
        if map_cache["key"] != key:
            rs = getattr(Image, "Resampling", Image)
            pil = Image.fromarray(img).resize(
                (max(1, int(w * eff)), max(1, int(h * eff))),
                rs.BICUBIC if s > 1 else rs.LANCZOS)
            map_cache["key"] = key
            map_cache["photo"] = ImageTk.PhotoImage(pil)
    else:
        # Pillow yoksa: tam sayı katlarıyla (bloklu) çizim
        if s >= 1:
            z, k, eff = int(s), 1, float(int(s))
        else:
            z, k = 1, math.ceil(1.0 / s)
            eff = 1.0 / k
        key = (ver, z, k)
        if map_cache["key"] != key:
            ppm = b"P6 %d %d 255\n" % (w, h) + img.tobytes()
            photo = tk.PhotoImage(data=ppm, format="PPM")
            if z > 1:
                photo = photo.zoom(z)
            if k > 1:
                photo = photo.subsample(k)
            map_cache["key"] = key
            map_cache["photo"] = photo

    x0 = (cw - w * eff) / 2
    y0 = (ch - h * eff) / 2
    c.create_image(x0, y0, image=map_cache["photo"], anchor="nw")
    map_geom.update(x0=x0, y0=y0, eff=eff, w=w, h=h, res=res, ox=ox, oy=oy)

    def to_px(wx, wy):
        return (x0 + ((wx - ox) / res) * eff,
                y0 + (h - (wy - oy) / res) * eff)

    if len(trail) > 1:
        pts = []
        for tx, ty in trail:
            pts.extend(to_px(tx, ty))
        c.create_line(*pts, fill="#3498db", width=2)

    if nav["path"]:
        pp_ = []
        for qx, qy in nav["path"]:
            pp_.extend(to_px(qx, qy))
        if len(pp_) >= 4:
            c.create_line(*pp_, fill="#f39c12", width=2, dash=(4, 3))
    if nav["goal_eff"] is not None:
        gx_, gy_ = to_px(*nav["goal_eff"])
        c.create_oval(gx_ - 7, gy_ - 7, gx_ + 7, gy_ + 7, outline="#e67e22", width=3)
        c.create_line(gx_ - 10, gy_, gx_ + 10, gy_, fill="#e67e22", width=2)
        c.create_line(gx_, gy_ - 10, gx_, gy_ + 10, fill="#e67e22", width=2)

    pose = ros_bridge.get_pose()
    if pose is not None:
        px, py = to_px(pose[0], pose[1])
        yaw = pose[2]
        c.create_oval(px - 6, py - 6, px + 6, py + 6, fill="#e74c3c", outline="")
        c.create_line(px, py, px + 16 * math.cos(yaw), py - 16 * math.sin(yaw),
                      fill="#e74c3c", width=3, arrow=tk.LAST)

    scans = ros_bridge.scan_count if ros_bridge else 0
    c.create_text(8, 8, anchor="nw", fill="#7f8c8d", font=("Arial", 9),
                  text=f"{w}x{h} cells  |  {res:.2f} m/cell  |  published scans: {scans}")

def update_loop():
    try:
        # --- Bağlantı sağlığı (LiDAR akışı ESP32 bağlantısının göstergesi) ---
        age = time.time() - last_scan_time
        if not lidar_seen:
            link_label.config(text="Link: waiting for LiDAR data", fg="#7f8c8d")
        elif age < LINK_WARN_S:
            link_label.config(text="Link: good", fg="#27ae60")
        elif age < LINK_LOST_S:
            link_label.config(text=f"Link: WEAK ({age:.1f} s without data)", fg="#e67e22")
        else:
            link_label.config(text=f"Link: LOST ({age:.0f} s) -> robot stopped", fg="#e74c3c")
            if STOP_ON_LINK_LOSS and lidar_seen and (is_autonomous or nav["active"]) \
                    and (current_left or current_right):
                stop_hard()

        # --- Otonom karar (sadece otonom modda) ---
        if is_autonomous:
            status_label.config(text=reactive_step())
        elif nav["active"]:
            status_label.config(text=nav_step())
        elif calib["active"]:
            status_label.config(text=calib_step())
        elif last_cmd_text:
            if not status_label.cget("text").startswith("Autonomous stopped") or current_left or current_right:
                status_label.config(text=last_cmd_text)

        motor_keepalive()
        motor_label.config(text=f"Left: {current_left}   Right: {current_right}")

        if is_autonomous and time.time() < stuck_display_until:
            stall_indicator.config(text="STATUS: STUCK", fg="#e74c3c")
        else:
            stall_indicator.config(text="STATUS: PHYSICAL MOTION OK", fg="#27ae60")

        update_trail()

        if notebook.index("current") == 0:
            draw_radar()
        else:
            draw_map()
    except Exception as e:
        print(f"Döngü hatası: {e}")
        stop()
    finally:
        if running:
            root.after(50, update_loop)

def on_close():
    global running, is_autonomous
    is_autonomous = False
    running = False
    stop_hard()
    stop_mapping()
    if ros_bridge is not None:
        ros_bridge.shutdown()
    motor_sock.close()
    root.destroy()

root.bind("<KeyPress>", key_press)
root.bind("<KeyRelease>", key_release)
root.bind("<space>", lambda event: toggle_mode())

def on_focus_out(event):
    """Pencere odağı kaybolunca tuş bırakma olayı gelmez; basılı tuş kalmasın diye manuel hareketi durdur."""
    if event.widget is root and pressed_keys:
        pressed_keys.clear()
        if not is_autonomous and not nav["active"] and not calib["active"]:
            stop_hard()

root.bind("<FocusOut>", on_focus_out)
root.protocol("WM_DELETE_WINDOW", on_close)

if ROS_OK:
    try:
        ros_bridge = RosBridge()
    except Exception as e:
        print(f"ROS köprüsü başlatılamadı: {e}")
        ros_bridge = None
        map_btn.config(state="disabled")
        map_info.config(text="ROS bridge failed to start")

threading.Thread(target=lidar_receiver, daemon=True).start()
update_loop()
stop()  # IP tanıtma pingi

root.focus_force()
root.mainloop()
