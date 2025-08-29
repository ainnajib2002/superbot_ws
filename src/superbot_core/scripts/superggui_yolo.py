# superggui_yolo.py – all‑in‑one Tkinter GUI + ROS2 integration
# ---------------------------------------------------------------
# 1) Build the GUI (main thread)
# 2) Run ROS2 executor in a background thread
# 3) Gesture STOP / GO / FOLLOW_MODE from /yolo/gesture_detected control the robot
# ---------------------------------------------------------------

# ============== standard libs ==============
import os
import sys
import math
import time
import threading
import tkinter as tk
from tkinter import PhotoImage, messagebox, simpledialog

# ============== ROS 2 =======================
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

# ============== Serial for load‑cell ========
import serial

# -------------------------------
# Load‑cell configuration constants
# -------------------------------
BAUDRATE = 115200
PORT     = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0"
MAX_WARNING = 37.0  # kg
MAX_LOAD    = 40.0  # kg

# -------------------------------
# Global runtime objects (populated later)
# -------------------------------
robot_commander = None
root = None
status_label = None
listbox_goals = None

# -------------------------------
# Navigation bookkeeping (GUI side)
# -------------------------------
navigasi_berjalan  = False
current_kategori   = None
selected_categories = []
kategori_buttons = []
ordered_categories = []

# -------------------------------
# Goal pose mapping (category → x,y,z,w)
# -------------------------------
goal_pose_map = {
    "Makanan":               (-3.03858, -0.862622, -0.750868, 0.660452),
    "Minuman":               ( 0.99505, -1.99781, -0.709441, 0.704765),
    "Produk Rumah Tangga":   (-3.96126, -3.98628, -0.680268, 0.732964),
    "Kesehatan & Kecantikan":( 1.01981, -4.06503, -0.772894, 0.634536),
    "Snack":                 (-2.00384,  1.04532,  0.777005, 0.629494),
    "Bahan Dapur":           ( 2.02745, -0.960099, 0.0921411, 0.995746),
    "Perawatan Bayi":        ( 1.00229,  1.53366, -0.800424, 0.599434),
    "Elektronik":            (-1.00817, -5.4164 , -0.0551012,0.998481),
    "Posisi Awal":           ( 0.0    ,  0.0    ,  0.0,      1.0)
}

bagian_kategori = {
    "Bagian A": ["Makanan", "Minuman"],
    "Bagian B": ["Produk Rumah Tangga", "Kesehatan & Kecantikan"],
    "Bagian C": ["Snack", "Bahan Dapur"],
    "Bagian D": ["Perawatan Bayi", "Elektronik"],
}

# ==============================================================
# RobotCommander node
# ==============================================================

class RobotCommander(Node):
    """Node yang menerima gesture dan mengelola mode GUIDE / STOP / FOLLOW"""

    MODE_GUIDE  = "GUIDE"
    MODE_STOP   = "STOP"
    MODE_FOLLOW = "FOLLOW"

    def __init__(self, tk_root, status_label):
        super().__init__('robot_commander')
        # GUI references (for thread‑safe update via after)
        self.tk_root = tk_root
        self.status_label = status_label

        # Publishers
        self.goal_pub  = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.twist_pub = self.create_publisher(Twist,       '/cmd_vel',  10)

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(String, '/yolo/gesture_detected', self.gesture_callback, 10)

        # Action / service clients
        self.nav_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_param_cli = self.create_client(SetParameters, '/follow_person/set_parameters')

        # Internal state
        self.mode = self.MODE_STOP
        self.goal_pose = None
        self.current_pose = (0.0, 0.0)

        self.get_logger().info('Robot Commander Node started.')

    # ----------------------------------------------------------
    # Public API (dipanggil GUI)
    # ----------------------------------------------------------

    def send_goal_pose(self, x, y, z, w):
        """Publish goal pose ke Nav2"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self.goal_pub.publish(pose)
        self.goal_pose = (x, y)
        self.get_logger().info(f"Sent goal pose → ({x:.2f}, {y:.2f})")

    # ----------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------

    def gesture_callback(self, msg):
        g = msg.data.upper().strip()
        self.get_logger().info(f"Gesture diterima: {g}")
        if g == "STOP":
            self._handle_stop()
        elif g == "GO":
            self._handle_go()
        elif g in ("FOLLOW_MODE", "HUMAN_FOLLOW"):
            self._handle_follow()

    def pose_callback(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.goal_pose is None:
            return
        dist = math.hypot(self.current_pose[0] - self.goal_pose[0],
                          self.current_pose[1] - self.goal_pose[1])
        if dist < 0.30:
            self.goal_pose = None
            self.get_logger().info("Robot telah sampai di tujuan")
            self._status_gui("Robot telah sampai di tujuan!")
            self.tk_root.after(500, cek_kedatangan)

    # ----------------------------------------------------------
    # Gesture handlers
    # ----------------------------------------------------------

    def _handle_stop(self):
        global navigasi_berjalan
        if self.mode == self.MODE_STOP:
            return
        # Cancel Nav2 goal (opsional)
        if self.nav_action.wait_for_server(timeout_sec=0.1):
            self.nav_action.cancel_all_goals_async()
        # Publish Twist nol
        self.twist_pub.publish(Twist())
        # Matikan follow
        self._set_follow_active(False)
        navigasi_berjalan = False
        self.mode = self.MODE_STOP
        self._status_gui("STOP → robot berhenti")

    def _handle_go(self):
        global navigasi_berjalan
        if self.mode != self.MODE_STOP:
            return  # harus stop dulu
        self.mode = self.MODE_GUIDE
        self._set_follow_active(False)
        navigasi_berjalan = False  # reset flag
        # Panggil logika tombol GO di thread GUI
        self.tk_root.after(0, start_navigation)
        self._status_gui("GO → lanjut navigasi")

    def _handle_follow(self):
        if self.mode != self.MODE_STOP:
            return
        self.mode = self.MODE_FOLLOW
        self._set_follow_active(True)
        self._status_gui("FOLLOW_MODE → mengikuti pengguna")

    # ----------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------

    def _set_follow_active(self, active):
        if not self.follow_param_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Service /follow_person/set_parameters tidak tersedia")
            return
        from rcl_interfaces.msg import Parameter, ParameterValue
        req = SetParameters.Request()
        param = Parameter(name="following_active",
                          value=ParameterValue(type=ParameterValue.TYPE_BOOL, bool_value=active))
        req.parameters = [param]
        self.follow_param_cli.call_async(req)

    def _status_gui(self, text):
        if self.status_label:
            self.tk_root.after(0, lambda: self.status_label.config(text=text))

# ==============================================================
# Utility functions (GUI side)
# ==============================================================

def urutkan_berdasarkan_jarak():
    if robot_commander is None:
        return
    pos_sekarang = robot_commander.current_pose
    selected_categories.sort(
        key=lambda k: math.hypot(pos_sekarang[0] - goal_pose_map[k][0],
                                 pos_sekarang[1] - goal_pose_map[k][1]))

def kirim_goal(kategori):
    if kategori not in goal_pose_map:
        status_label.config(text="Kategori tidak valid")
        return
    if kategori in selected_categories:
        status_label.config(text=f"'{kategori}' sudah ada di daftar")
        return
    selected_categories.append(kategori)
    listbox_goals.insert(tk.END, kategori)
    status_label.config(text=f"'{kategori}' ditambahkan ke daftar.")


def start_navigation():
    global navigasi_berjalan, current_kategori, ordered_categories
    if not selected_categories:
        status_label.config(text="Daftar pembelian kosong.")
        return
    urutkan_berdasarkan_jarak()
    ordered_categories = selected_categories.copy()
    current_kategori = ordered_categories[0]
    pose = goal_pose_map[current_kategori]
    status_label.config(text=f"Menuju rak '{current_kategori}' …")
    # highlight listbox
    for i in range(listbox_goals.size()):
        if listbox_goals.get(i) == current_kategori:
            listbox_goals.selection_clear(0, tk.END)
            listbox_goals.selection_set(i)
            break
    navigasi_berjalan = True
    if robot_commander:
        robot_commander.send_goal_pose(*pose)


def cek_kedatangan():
    global navigasi_berjalan, current_kategori, ordered_categories
    if robot_commander.goal_pose is None:
        # mark visited
        for i in range(listbox_goals.size()):
            if listbox_goals.get(i) == current_kategori:
                listbox_goals.itemconfig(i, {'bg': '#90ee90'})
                listbox_goals.delete(i)
                break
        status_label.config(text=f"Tiba di '{current_kategori}'. Tekan GO untuk lanjut.")
        if current_kategori in selected_categories:
            selected_categories.remove(current_kategori)
        if ordered_categories:
            ordered_categories.pop(0)
        navigasi_berjalan = False
        if not ordered_categories:
            status_label.config(text="Semua tujuan telah dikunjungi.")
            for btn in kategori_buttons:
                btn.config(state=tk.NORMAL)
    else:
        root.after(500, cek_kedatangan)


def kembali_ke_awal():
    if robot_commander:
        pose = goal_pose_map["Posisi Awal"]
        robot_commander.send_goal_pose(*pose)
        status_label.config(text="Kembali ke posisi awal …")


def hapus_kategori():
    sel = listbox_goals.curselection()
    if not sel:
        status_label.config(text="Pilih kategori yang akan dihapus.")
        return
    kat = listbox_goals.get(sel)
    selected_categories.remove(kat)
    listbox_goals.delete(sel)
    status_label.config(text=f"'{kat}' dihapus dari daftar.")

# ==============================================================
# LoadcellApp – unchanged logic except minor tweaks
# ==============================================================

class LoadcellApp:
    def __init__(self, root):
        self.root = root
        self.blinking = False
        self.blink_state = True
        # … GUI widgets …
        tk.Label(root, text="Data Loadcell", font=("Helvetica", 14, "bold"), bg="#f0f8ff", fg="#2f4f4f").pack(pady=(25, 0))
        grid_frame = tk.Frame(root, bg="#f0f8ff")
        grid_frame.pack(pady=5)
        # Berat
        berat_frame = tk.LabelFrame(grid_frame, text="Berat", font=("Arial", 10, "bold"), bg="#f0f8ff", fg="#2f4f4f")
        berat_frame.grid(row=0, column=0, padx=8, pady=8, sticky="n")
        self.weight_btn = tk.Button(berat_frame, text="-- kg", font=("Arial", 11, "bold"), width=20, bg="#f0f8ff", relief="solid", bd=2)
        self.weight_btn.pack(padx=10, pady=1)
        # Status
        status_frame = tk.LabelFrame(grid_frame, text="Status", font=("Arial", 10, "bold"), bg="#f0f8ff", fg="#2f4f4f")
        status_frame.grid(row=0, column=1, padx=8, pady=8, sticky="n")
        self.status_btn = tk.Button(status_frame, text="--", font=("Arial", 11, "bold"), width=20, bg="gray", fg="white", relief="solid", bd=2)
        self.status_btn.pack(padx=10, pady=1)
        # Reset
        tk.Button(root, text="Reset", font=("Arial", 11, "bold"), width=18, bg="#4682b4", fg="white", command=self.reset_status).pack(pady=5)
        # Serial init
        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
            time.sleep(2)
            self.ser.reset_input_buffer()
        except serial.SerialException:
            self.ser = None
            print("[INFO] : ESP32 belum connect")
            self.update_status("Port belum connect", "red")
        # Thread baca
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ------------------ serial helpers ------------------
    def _cmd(self, c):
        if not self.ser:
            return ""
        try:
            self.ser.reset_input_buffer()
            self.ser.write(c.encode() + b'\r')
            time.sleep(0.05)
            return self.ser.readline().decode().strip()
        except Exception:
            return ""

    def _read_loop(self):
        while self.running:
            raw = self._cmd('f')
            try:
                val = float(raw)
                self._update_weight(val)
            except ValueError:
                self._update_weight(None)
            time.sleep(0.1)

    # ------------------ UI update helpers ----------------
    def _update_weight(self, kg):
        if kg is None:
            self.root.after(0, lambda: self.weight_btn.config(text="Data tidak valid", bg="#f0f8ff", fg="black"))
            self.root.after(0, lambda: self.update_status("Tidak Valid", "gray"))
            self.blinking = False
            return
        self.root.after(0, lambda: self.weight_btn.config(text=f"{kg:.2f} kg"))
        if kg > MAX_LOAD:
            self.root.after(0, lambda: self.weight_btn.config(bg="red", fg="white"))
            self.root.after(0, lambda: self.update_status("Melebihi Kapasitas", "red"))
            self.blinking = False
        elif kg > MAX_WARNING:
            self.root.after(0, lambda: self.weight_btn.config(bg="orange", fg="black"))
            self.root.after(0, lambda: self.update_status("Warning", "orange"))
            if not self.blinking:
                self.blinking = True
                self._blink()
        else:
            self.root.after(0, lambda: self.weight_btn.config(bg="green", fg="white"))
            self.root.after(0, lambda: self.update_status("Aman", "green"))
            self.blinking = False

    def update_status(self, text, color):
        self.status_btn.config(text=text, bg=color)

    def _blink(self):
        if not self.blinking:
            return
        color = "orange" if self.weight_btn.cget("bg") == "white" else "white"
        self.status_btn.config(bg=color)
        self.root.after(500, self._blink)

    def reset_status(self):
        self.weight_btn.config(text="-- kg", bg="#f0f8ff", fg="black")
        self.status_btn.config(text="--", bg="gray", fg="white")
        self.blinking = False
        self._cmd('t')

    def close(self):
        self.running = False
        time.sleep(0.2)
        if self.ser:
            self.ser.close()
        self.root.destroy()

# ==============================================================
# GUI construction (must run in main thread)
# ==============================================================

def main():
    global root, status_label, listbox_goals
    root = tk.Tk()
    root.title("Robot Supermarket")
    root.configure(bg="#f0f8ff")
    root.geometry("600x1000")
    root.resizable(True, True)

    ikon_path = "/home/orin/superbot_ws/src/superbot_core/superbot.png"
    if os.path.exists(ikon_path):
        try:
            root.iconphoto(True, PhotoImage(file=ikon_path))
        except Exception as e:
            print(f"[!] Gagal memuat ikon: {e}")

    judul_label = tk.Label(root, text="Navigasi Robot Supermarket", font=("Helvetica", 14, "bold"), bg="#f0f8ff", fg="#2f4f4f")
    judul_label.pack(pady=(10, 5))

    # ------- kategori buttons -------
    kategori_frame = tk.Frame(root, bg="#f0f8ff")
    kategori_frame.pack(padx=10)

    urutan_grid = [("Bagian A", 0, 0), ("Bagian B", 0, 1), ("Bagian C", 1, 0), ("Bagian D", 1, 1)]
    for bagian, r, c in urutan_grid:
        lf = tk.LabelFrame(kategori_frame, text=bagian, font=("Arial", 10, "bold"), bg="#f0f8ff", fg="#2f4f4f")
        lf.grid(row=r, column=c, padx=8, pady=8, sticky="n")
        for kat in bagian_kategori[bagian]:
            btn = tk.Button(lf, text=kat, width=22, font=("Arial", 9), bg="#4682b4", fg="white",
                            command=lambda k=kat: kirim_goal(k))
            btn.pack(pady=2)
            kategori_buttons.append(btn)

    # ------- listbox goals -------
    listbox_frame = tk.Frame(root, bg="#f0f8ff")
    listbox_frame.pack(pady=9, padx=10)

    tk.Label(listbox_frame, text="Barang yang ingin dibeli:", font=("Arial", 10, "bold"), bg="#f0f8ff", fg="#2f4f4f").pack()

    listbox_goals = tk.Listbox(listbox_frame, width=40, height=8, font=("Arial", 10))
    listbox_goals.pack()

    # ------- control buttons -------
    btn_frame = tk.Frame(root, bg="#f0f8ff")
    btn_frame.pack(pady=(0, 8))

    tk.Button(btn_frame, text="GO",              width=18, font=("Arial", 11, "bold"), bg="#32cd32", fg="white", command=start_navigation).grid(row=0, column=0, padx=5)

    tk.Button(btn_frame, text="Kembali ke Awal", width=18, font=("Arial", 11, "bold"), bg="#ff8c00", fg="white", command=kembali_ke_awal).grid(row=0, column=1, padx=5)

    tk.Button(btn_frame, text="Hapus",           width=18, font=("Arial", 11, "bold"), bg="#ff4500", fg="white", command=hapus_kategori).grid(row=0, column=2, padx=5)

    status_label = tk.Label(root, text="Klik kategori untuk memulai navigasi.", font=("Arial", 10, "italic"), bg="#f0f8ff", fg="green")
    status_label.pack(pady=(0, 8))

    # ------- Loadcell widget -------
    loadcell_app = LoadcellApp(root)

    # ==============================================================
    # Start ROS in background thread
    # ==============================================================

    def ros_spin_thread():
        global robot_commander
        rclpy.init()
        robot_commander = RobotCommander(tk_root=root, status_label=status_label)
        executor = MultiThreadedExecutor()
        executor.add_node(robot_commander)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            robot_commander.destroy_node()
            rclpy.shutdown()

    threading.Thread(target=ros_spin_thread, daemon=True).start()

    # ==============================================================
    # Graceful shutdown
    # ==============================================================

    def on_closing():
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()
        sys.exit()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

# ==============================================================
# Mainloop
# ==============================================================

if __name__ == '__main__':
    main()