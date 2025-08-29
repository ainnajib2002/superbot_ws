from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry # Import pesan Odometry
from std_msgs.msg import String

import tkinter as tk
from tkinter import PhotoImage
import os
import rclpy
from rclpy.node import Node
from threading import Thread
import math
import sys
import serial
import time
import threading
from tkinter import simpledialog, messagebox
from std_msgs.msg import Float32  # Import Float32 message type
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue, ParameterType
from rclpy.parameter import Parameter as RosParameter # Beri alias untuk menghindari konflik

# -------------------------------
# Load Cell Configuration
# -------------------------------
BAUDRATE = 115200
PORT = "/dev/serial/by-path/platform-3610000.xhci-usb-0:2.3:1.0-port0"
MAX_WARNING = 26.0
MAX_LOAD = 30.0

# -------------------------------
# Global Variables
# -------------------------------
robot_commander = None
status_label = None
navigasi_berjalan = False
current_kategori = None
selected_categories = []
kategori_buttons = []
ordered_categories = []
# --- BARU: Variabel untuk state gestur ---
robot_stopped = False
follow_mode = False

# -------------------------------
# Dictionary for goal poses based on categories
# -------------------------------
goal_pose_map = {
    "A1": (0.20, -3.00, -0.707, 0.707),
    "A2": (2.00, -3.20, 0.707, 0.707),
    "B1": (3.00, -3.20, 0.707, 0.707),
    "B2": (5.20, -3.00, -0.707, 0.707),
    "Posisi Awal": (0.0, 0.0, 0.0, 1.0)
}

bagian_kategori = {
    "Rack A": ["A1", "A2"],
    "Rack B": ["B1", "B2"],
}

# -------------------------------
# RobotCommander Class (ROS2 Node)
# -------------------------------
class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_commander')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.loadcell_publisher_ = self.create_publisher(Float32, '/loadcell', 10)
        # --- BARU: Publisher untuk menghentikan robot ---
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # --- BARU: Subscriber untuk deteksi gestur dari YOLO ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.gesture_subscription = self.create_subscription(
            String,
            '/yolo/gesture_detected',
            self.gesture_callback,
            qos_profile
        )

        self.goal_pose = None
        self.current_pose = (0.0, 0.0)
        self.last_movement_time = self.get_clock().now()
        self.is_navigating_to_goal = False

        self.movement_monitor_thread = threading.Thread(target=self.monitor_movement, daemon=True)
        self.movement_monitor_thread.start()

        # --- BARU: Client untuk mengaktifkan/menonaktifkan mode follow ---
        self.param_client = self.create_client(SetParameters, '/follow_person/set_parameters')
        
        self.get_logger().info('Robot Commander Node started.')
        self.get_logger().info('Gesture subscriber created for topic: /yolo/gesture_detected')
        self.get_logger().info("Service client for '/follow_person/set_parameters' created.")


    def send_goal_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self.publisher_.publish(pose)
        
        self.get_logger().info(f"send_goal_pose: Goal pose {x:.2f},{y:.2f} DIPUBLIKASIKAN ke /goal_pose.")

        self.goal_pose = (x, y)
        self.is_navigating_to_goal = True
        self.last_movement_time = self.get_clock().now()
        self.get_logger().info(f"Sent goal pose to x={x}, y={y}, z={z}, w={w}. is_navigating_to_goal={self.is_navigating_to_goal}")

    # --- BARU: Fungsi untuk mengirim perintah berhenti ---
    def stop_robot(self):
        stop_msg = Twist()
        # Set semua kecepatan menjadi nol
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info("Robot stopped via cmd_vel by gesture")

    def odom_callback(self, msg: Odometry):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        linear_speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        angular_speed = msg.twist.twist.angular.z

        movement_threshold_odom = 0.01

        if linear_speed > movement_threshold_odom or abs(angular_speed) > movement_threshold_odom:
            self.last_movement_time = self.get_clock().now()

        if self.goal_pose is not None:
            dist = math.sqrt((self.current_pose[0] - self.goal_pose[0])**2 +
                             (self.current_pose[1] - self.goal_pose[1])**2)
            
            # Kurangi frekuensi logging untuk kebersihan terminal
            # self.get_logger().info(f"odom_callback: Jarak ke tujuan ({self.goal_pose[0]:.2f},{self.goal_pose[1]:.2f}): {dist:.2f} m")

            if dist < 0.3:
                self.get_logger().info("Robot sangat dekat dengan tujuan. Menunggu konfirmasi berhenti dari monitor_movement.")

    def publish_loadcell_data(self, weight):
        msg = Float32()
        msg.data = weight
        self.loadcell_publisher_.publish(msg)

    def monitor_movement(self):
        while rclpy.ok():
            time.sleep(0.01) 
            if self.goal_pose is None and not self.is_navigating_to_goal:
                continue 
            
            elapsed = self.get_clock().now() - self.last_movement_time
            self.get_logger().info(f"monitor_movement: Waktu berlalu sejak gerakan terakhir: {elapsed.nanoseconds / 1e9:.2f} s. Navigating: {self.is_navigating_to_goal}")

            if elapsed.nanoseconds > 3 * 1e9 and self.is_navigating_to_goal:  
                self.get_logger().warn("Robot dianggap telah sampai karena tidak ada pergerakan selama 3 detik.")
                if self.goal_pose is not None: 
                    self.goal_pose = None 
                    self.is_navigating_to_goal = False 
                    if status_label:
                        status_label.after(0, lambda: status_label.config(text="Robot dianggap telah sampai karena tidak ada pergerakan."))
                        status_label.after(10, cek_kedatangan) 
                else:
                    self.get_logger().info("monitor_movement: Goal_pose sudah None, tapi is_navigating_to_goal masih True. Mereset flag.")
                    self.is_navigating_to_goal = False
 
    # --- BARU: Fungsi untuk mengaktifkan/menonaktifkan mode follow ---
    def activate_follow_mode(self, active: bool):
        if not self.param_client.service_is_ready():
            self.get_logger().error("Service '/follow_person/set_parameters' tidak aktif.")
            return

        req = SetParameters.Request()
        param = Parameter()
        param.name = "following_active"
        param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=active)
        req.parameters.append(param)
        
        self.param_client.call_async(req)
        self.get_logger().info(f"Perintah untuk mengatur mode follow ke {active} telah dikirim.")

    # --- BARU: Callback untuk memproses perintah gestur ---
    def gesture_callback(self, msg):
        global robot_stopped, follow_mode, navigasi_berjalan
        gesture = msg.data.strip()
        
        if gesture == "STOP":
            robot_stopped = True
            follow_mode = False
            self.stop_robot()
            self.activate_follow_mode(False)
            status_label.config(text="ROBOT STOP.", fg="red")
            
        elif gesture == "GO":
            if not selected_categories and not ordered_categories:
                status_label.config(text="EMPTY LIST, PLEASE INSERT SOME.", fg="orange")
                return
                
            robot_stopped = False
            follow_mode = False
            self.activate_follow_mode(False) 
            status_label.config(text="GO, CONTINUE NAVIGATION.", fg="green")
            start_navigation() # Memanggil fungsi navigasi utama
            
        elif gesture == "FOLLOW_MODE":
            robot_stopped = False
            follow_mode = True
            self.stop_robot() # Hentikan navigasi goal-based
            navigasi_berjalan = False
            self.is_navigating_to_goal = False # Hentikan monitor
            self.goal_pose = None
            self.activate_follow_mode(True)
            status_label.config(text="FOLLOW. HUMAN_FOLLOW MODE ACTIVE.", fg="blue")



# -------------------------------
# Utility Function: Sort by Distance
# -------------------------------
def urutkan_berdasarkan_jarak():
    if robot_commander is None:
        return
    posisi_sekarang = robot_commander.current_pose
    selected_categories.sort(
        key=lambda kategori: math.sqrt(
            (posisi_sekarang[0] - goal_pose_map[kategori][0])**2 +
            (posisi_sekarang[1] - goal_pose_map[kategori][1])**2
        )
    )

# -------------------------------
# Send Category to List and Navigate
# -------------------------------
def kirim_goal(kategori):
    if kategori in goal_pose_map:
        if kategori not in selected_categories:
            selected_categories.append(kategori)
            listbox_goals.insert(tk.END, kategori)
            status_label.config(text=f"'{kategori}' has been added to the shopping list.")
        else:
            status_label.config(text=f"'{kategori}' is already in the shopping list.")
    else:
        status_label.config(text="Invalid category.")


# -------------------------------
# Navigate to the First Category
# -------------------------------
def start_navigation():
    global navigasi_berjalan, current_kategori, ordered_categories

    if not selected_categories:
        status_label.config(text="The shopping list is empty.")
        return

    urutkan_berdasarkan_jarak()
    ordered_categories = selected_categories.copy()

    btn_go.config(text="Next") 

    current_kategori = ordered_categories[0]
    pose = goal_pose_map[current_kategori]
    status_label.config(text=f"Navigating to rack '{current_kategori}'...")

    for i in range(listbox_goals.size()):
        if listbox_goals.get(i) == current_kategori:
            listbox_goals.selection_clear(0, tk.END)
            listbox_goals.selection_set(i)
            break

    navigasi_berjalan = True

    if robot_commander:
        # --- DEBUG LOGS: Periksa state sebelum mengirim goal baru ---
        robot_commander.get_logger().info(f"start_navigation: Memulai navigasi ke '{current_kategori}'.")
        robot_commander.get_logger().info(f"start_navigation: Goal pose baru: x={pose[0]:.2f}, y={pose[1]:.2f}")
        robot_commander.get_logger().info(f"start_navigation: State sebelum send_goal_pose: goal_pose={robot_commander.goal_pose}, is_navigating_to_goal={robot_commander.is_navigating_to_goal}")
        
        robot_commander.send_goal_pose(*pose)

def cek_kedatangan():
    global navigasi_berjalan, current_kategori, ordered_categories

    if robot_commander.goal_pose is None:
        found_in_listbox = False
        for i in range(listbox_goals.size()):
            if listbox_goals.get(i) == current_kategori:
                listbox_goals.itemconfig(i, {'bg': '#90ee90'})
                listbox_goals.delete(i)
                found_in_listbox = True
                break
        
        if not found_in_listbox:
            robot_commander.get_logger().warn(f"cek_kedatangan: current_kategori '{current_kategori}' tidak ditemukan di listbox. Mungkin sudah dihapus secara manual atau karena masalah lain.")

        status_label.config(text=f"The robot has arrived at '{current_kategori}'. Press next to continue.")

        if current_kategori in selected_categories:
            selected_categories.remove(current_kategori)
        if ordered_categories:
            ordered_categories.pop(0)

        navigasi_berjalan = False

        if not ordered_categories:
            status_label.config(text="All destinations have been visited.")
            btn_go.config(text="GO") # Kembalikan juga tombol GO di sini
            for btn in kategori_buttons:
                    btn.config(state=tk.NORMAL)
    else:
        robot_commander.get_logger().info("cek_kedatangan: Dipanggil tapi goal_pose masih aktif. Menunggu.")
        root.after(500, cek_kedatangan)

def kembali_ke_awal():
    if robot_commander:
        pose = goal_pose_map["Posisi Awal"]
        robot_commander.send_goal_pose(*pose)
        status_label.config(text="Returning to the starting position...")

def hapus_kategori():
    selected_index = listbox_goals.curselection()
    if not selected_index:
        status_label.config(text="Please select a category to remove.")
        return

    kategori_to_remove = listbox_goals.get(selected_index)
    selected_categories.remove(kategori_to_remove)
    listbox_goals.delete(selected_index)
    status_label.config(text=f"'{kategori_to_remove}' has been removed from the shoppping list.")

# -------------------------------
# ROS2 Thread
# -------------------------------
def ros_main():
    global robot_commander
    rclpy.init()
    robot_commander = RobotCommander()
    try:
        rclpy.spin(robot_commander)
    except KeyboardInterrupt:
        pass
    finally:
        robot_commander.destroy_node()
        rclpy.shutdown()

ros_thread = Thread(target=ros_main, daemon=True)
ros_thread.start()

# -------------------------------
# GUI Tkinter Setup
# -------------------------------
root = tk.Tk()
root.title("Robot Supermarket")
root.configure(bg="#f0f8ff")
root.geometry("600x1000")  # Adjusted height to accommodate LoadcellApp lebar x tinggi
root.resizable(True, True)

ikon_path = "/home/orin/superbot_ws/src/superbot_core/superbot.png"
if os.path.exists(ikon_path):
    try:
        icon = PhotoImage(file=ikon_path)
        root.iconphoto(True, icon)
    except Exception as e:
        print(f"[!] Gagal memuat ikon: {e}")

judul_label = tk.Label(root, text="Supermarket Robot Navigation",
                       font=("Helvetica", 16, "bold"),
                       bg="#f0f8ff", fg="#2f4f4f")
judul_label.pack(pady=(10, 5))

kategori_frame = tk.Frame(root, bg="#f0f8ff")
kategori_frame.pack(padx=10)

urutan_bagian = [("Rack A", 0, 0),
                 ("Rack B", 0, 1)]

for bagian, row, col in urutan_bagian:
    frame = tk.LabelFrame(kategori_frame, text=bagian,
                          font=("Arial", 13, "bold"),
                          bg="#f0f8ff", fg="#2f4f4f",
                          labelanchor="n", padx=8, pady=4, width=190)
    frame.grid(row=row, column=col, padx=4, pady=4, sticky="n")
    for kategori in bagian_kategori[bagian]:
        btn = tk.Button(frame, text=kategori, width=22,
                        font=("Arial", 12, "bold"),
                        bg="#4682b4", fg="white", padx=5, pady=3,
                        command=lambda k=kategori: kirim_goal(k))
        btn.pack(pady=2)
        kategori_buttons.append(btn)

listbox_frame = tk.Frame(root, bg="#f0f8ff")
listbox_frame.pack(pady=3, padx=10)

label_daftar = tk.Label(listbox_frame, text="Shopping list:",
                        font=("Arial", 12, "bold"),
                        bg="#f0f8ff", fg="#2f4f4f", anchor="center", justify="center")
label_daftar.pack()

listbox_goals = tk.Listbox(listbox_frame, width=30, height=4, font=("Arial", 12), justify="center")
listbox_goals.pack()

button_frame = tk.Frame(root, bg="#f0f8ff")
button_frame.pack(pady=(7, 8))

btn_go = tk.Button(button_frame, text="GO", width=18, font=("Arial", 12, "bold"),
                   bg="#32cd32", fg="white", padx=5, pady=5,
                   command=start_navigation)
btn_go.grid(row=0, column=0, padx=5)

btn_kembali = tk.Button(button_frame, text="Return", width=18, font=("Arial", 12, "bold"),
                        bg="#ff8c00", fg="white", padx=5, pady=5,
                        command=kembali_ke_awal)
btn_kembali.grid(row=0, column=1, padx=5)

status_label = tk.Label(root, text="Click a category to start navigation.",
                        font=("Arial", 12, "italic"),
                        bg="#f0f8ff", fg="green")
status_label.pack(pady=(5, 8))

btn_hapus = tk.Button(button_frame, text="Remove", width=18, font=("Arial", 12, "bold"),
                      bg="#ff4500", fg="white", padx=5, pady=5,
                      command=hapus_kategori)
btn_hapus.grid(row=0, column=2, padx=5)


# -------------------------------
# LoadcellApp Class
# # -------------------------------
class LoadcellApp:
    def __init__(self, root):
        self.root = root
        self.blinking = False
        self.blink_state = True

        self.title_label = tk.Label(root, text="Shopping Weight", font=("Helvetica", 16, "bold"),  bg="#f0f8ff", fg="#2f4f4f")
        self.title_label.pack(pady=(10, 0))  
        
        grid_frame = tk.Frame(root, bg="#f0f8ff")
        grid_frame.pack(pady=5)

        berat_frame = tk.LabelFrame(grid_frame, text="Weight", font=("Arial", 13, "bold"),
                                   bg="#f0f8ff", fg="#2f4f4f", labelanchor="n", padx=10, pady=4)
        berat_frame.grid(row=0, column=0, padx=0, pady=0, sticky="n")

        self.weight_value_button = tk.Button(
            berat_frame,
            text="-- kg",
            font=("Arial", 12, "bold"),  
            width=20,                    
            height=1,                    
            bg="#f0f8ff",
            fg="black",
            relief="solid",
            bd=2,
            anchor="center",
            justify="center"
        )
        self.weight_value_button.pack(padx=10, pady=1)

        status_frame = tk.LabelFrame(grid_frame, text="Status", font=("Arial", 13, "bold"),
                                    bg="#f0f8ff", fg="#2f4f4f", labelanchor="n", padx=10, pady=4)
        status_frame.grid(row=0, column=1, padx=0, pady=0, sticky="n")

        self.status_value_button = tk.Button(
            status_frame,
            text="--",
            font=("Arial", 12, "bold"),  
            width=20,                    
            height=1,                    
            bg="gray",
            fg="white",
            relief="solid",
            bd=2,
            anchor="center",
            justify="center",
            wraplength=300
        )
        self.status_value_button.pack(padx=10, pady=1)

        button_frame = tk.Frame(root, bg="#f0f8ff")
        button_frame.pack(pady=0)

        self.reset_button = tk.Button(
                button_frame,
                text="Reset",
                font=("Arial", 12, "bold"), 
                width=18,                   
                height=1,                   
                bg="#4682b4",             
                fg="white",
                command=self.reset_status,
                anchor="center",         
                justify="center"
            )
        self.reset_button.grid(row=0, column=0, padx=5, pady=0, sticky="nsew")

        self.warning_label = tk.Label(root, text="", font=("Helvetica", 9), bg="#f0f8ff", fg="green")
        self.warning_label.pack(pady=5)

        self.running = True
        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.send_command('l')
        except serial.SerialException:
            self.ser = None
            print("[INFO] : ESP32 Belum Connect")  
            self.update_status("[INFO] : Port belum dicolok atau belum connect", "red")
            
        self.thread = threading.Thread(target=self.read_weight_loop, daemon=True)
        self.thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def update_status(self, text, color):
        self.status_value_button.config(text=text, bg=color)

    def send_command(self, cmd):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode() + b'\r')
            time.sleep(0.05)
            response = self.ser.readline().decode().strip()
            return response
        except Exception as e:
            return ""

    def read_weight_loop(self):
        while self.running:
            line = self.send_command('f')
            try:
                berat = float(line)
                self.update_weight(berat)
                if robot_commander: 
                    robot_commander.publish_loadcell_data(berat) 
            except ValueError:
                self.update_weight(None)
            time.sleep(0.1)
            
    def update_weight(self, berat):
        if berat is None:
            self.root.after(0, lambda: self.weight_value_button.config(text="Data tidak valid", fg="black", bg="#f0f8ff"))
            self.root.after(0, lambda: self.update_status("Invalid data", "gray"))
            self.blinking = False
        else:
            self.root.after(0, lambda: self.weight_value_button.config(text=f"{berat:.2f} kg"))
            if berat > MAX_LOAD:
                self.root.after(0, lambda: self.weight_value_button.config(fg="white", bg="red"))
                self.root.after(0, lambda: self.update_status("Over capacity", "red"))
                self.blinking = False
            elif berat > MAX_WARNING:
                self.root.after(0, lambda: self.weight_value_button.config(fg="white", bg="orange"))
                self.root.after(0, lambda: self.update_status("Warning capacity", "orange"))
                if not self.blinking:
                    self.blinking = True
                    self.blink_warning()
            else:
                self.root.after(0, lambda: self.weight_value_button.config(fg="white", bg="green"))
                self.root.after(0, lambda: self.update_status("Normal Capacity", "green"))
                self.blinking = False

    def update_status(self, text, color):
        self.status_value_button.config(text=text, bg=color)

    def blink_warning(self):
        if self.blinking:
            current_color = "orange" if self.blink_state else "white"
            self.status_value_button.config(bg=current_color)
            self.blink_state = not self.blink_state
            self.root.after(500, self.blink_warning)
        else:
            self.status_value_button.config(bg="orange")

    def reset_status(self):
        self.weight_value_button.config(text="-- kg", bg="#f0f8ff", fg="black")
        self.status_value_button.config(text="--", bg="gray", fg="white")
        self.blinking = False
        try:
            self.send_command('t')
        except Exception as e:
            print("Gagal kirim perintah reset:", e)

    def close(self):
        self.running = False
        if self.ser:
            time.sleep(0.2)
            self.ser.close()
        self.root.destroy()

loadcell_app = LoadcellApp(root)

def on_closing():
    if rclpy.ok():
        rclpy.shutdown()
    root.destroy()
    sys.exit()

root.protocol("WM_DELETE_WINDOW", on_closing)
try:
    root.mainloop()
except KeyboardInterrupt:
    on_closing()