from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose  # Import the action type
import tkinter as tk
from tkinter import PhotoImage
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from threading import Thread
import math
import sys

# -------------------------------
# Variabel global
# -------------------------------
robot_commander = None
status_label = None
navigasi_berjalan = False
current_kategori = None
selected_categories = []
kategori_buttons = []
ordered_categories = []

# -------------------------------
# Dictionary goal pose berdasarkan kategori
# Format: x, y, orientation_z, orientation_w
# -------------------------------
goal_pose_map = {
    "Makanan": (-3.03858, -0.862622, -0.750868, 0.660452),
    "Minuman": (0.99505, -1.99781, -0.709441, 0.704765),
    "Produk Rumah Tangga": (-3.96126, -3.98628, -0.680268, 0.732964),
    "Kesehatan & Kecantikan": (1.01981, -4.06503, -0.772894, 0.634536),
    "Snack": (-2.00384, 1.04532, 0.777005, 0.629494),
    "Bahan Dapur": (2.02745, -0.960099, 0.0921411, 0.995746),
    "Perawatan Bayi": (1.00229, 1.53366, -0.800424, 0.599434),
    "Elektronik": (-1.00817, -5.4164, -0.0551012, 0.998481),
    "Posisi Awal": (0.0, 0.0, 0.0, 1.0)  # Tambahkan ini
}

bagian_kategori = {
    "Bagian A": ["Makanan", "Minuman"],
    "Bagian B": ["Produk Rumah Tangga", "Kesehatan & Kecantikan"],
    "Bagian C": ["Snack", "Bahan Dapur"],
    "Bagian D": ["Perawatan Bayi", "Elektronik"]
}

# -------------------------------
# Kelas RobotCommander (ROS2 Node)
# -------------------------------
class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_commander')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_pose = None
        self.current_pose = (0.0, 0.0)
        self.get_logger().info('Robot Commander Node started.')

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
        self.goal_pose = (x, y)
        self.get_logger().info(f"Sent goal pose to x={x}, y={y}, z={z}, w={w}")

        # Send the action goal
        self.send_navigation_goal(pose)

    def send_navigation_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info("Reached to goal.")  # Logger tambahan
            status_label.after(0, lambda: status_label.config(text=f"Robot telah sampai di '{current_kategori}'. Tekan GO untuk melanjutkan."))
            status_label.after(500, cek_kedatangan)  # Periksa tujuan berikutnya
        else:
            self.get_logger().error("Navigation failed.")
            status_label.after(0, lambda: status_label.config(text="Navigasi gagal. Silakan coba lagi."))

    def pose_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        if self.goal_pose is None:
            return
        dist = math.sqrt((self.current_pose[0] - self.goal_pose[0])**2 +
                        (self.current_pose[1] - self.goal_pose[1])**2)
        if dist < 0.3:  # Jika robot sudah dekat dengan tujuan
            self.goal_pose = None
            self.get_logger().info("Robot telah sampai di tujuan.")
            if status_label:
                status_label.after(0, lambda: status_label.config(text="Robot telah sampai di tujuan!"))
                status_label.after(500, cek_kedatangan)  # Periksa tujuan berikutnya

# -------------------------------
# Define Functions Before GUI Setup
# -------------------------------
def start_navigation():
    global navigasi_berjalan, current_kategori, ordered_categories

    if not selected_categories:
        status_label.config(text="Daftar pembelian kosong.")
        return

    urutkan_berdasarkan_jarak()
    ordered_categories = selected_categories.copy()

    current_kategori = ordered_categories[0]
    pose = goal_pose_map[current_kategori]
    status_label.config(text=f"Menuju rak '{current_kategori}'...")

    for i in range(listbox_goals.size()):
        if listbox_goals.get(i) == current_kategori:
            listbox_goals.selection_clear(0, tk.END)
            listbox_goals.selection_set(i)
            break

    navigasi_berjalan = True

    if robot_commander:
        robot_commander.send_goal_pose(*pose)

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

def kirim_goal(kategori):
    if kategori in goal_pose_map:
        if kategori not in selected_categories:
            selected_categories.append(kategori)
            listbox_goals.insert(tk.END, kategori)
            status_label.config(text=f"'{kategori}' ditambahkan ke daftar pembelian.")
        else:
            status_label.config(text=f"'{kategori}' sudah ada di daftar pembelian.")
    else:
        status_label.config(text="Kategori tidak valid.")

def cek_kedatangan():
    global navigasi_berjalan, current_kategori, ordered_categories

    if robot_commander.goal_pose is None:  # Robot telah mencapai tujuan
        # Cari indeks berdasarkan nama kategori
        for i in range(listbox_goals.size()):
            if listbox_goals.get(i) == current_kategori:
                listbox_goals.itemconfig(i, {'bg': '#90ee90'})  # Tandai dengan warna hijau
                listbox_goals.delete(i)  # Hapus dari listbox
                break

        status_label.config(text=f"Robot telah sampai di '{current_kategori}'.")

        # Hapus dari daftar
        if current_kategori in selected_categories:
            selected_categories.remove(current_kategori)
        if ordered_categories:
            ordered_categories.pop(0)

        navigasi_berjalan = False

        # # Jika masih ada tujuan berikutnya, kirimkan secara otomatis
        # if ordered_categories:
        #     current_kategori = ordered_categories[0]
        #     pose = goal_pose_map[current_kategori]
        #     status_label.config(text=f"Menuju rak '{current_kategori}'...")
        #     robot_commander.send_goal_pose(*pose)  # Kirim tujuan berikutnya tanpa delay
        # else:
        #     status_label.config(text="Semua tujuan telah dikunjungi.")
        #     for btn in kategori_buttons:
        #         btn.config(state=tk.NORMAL)  # Aktifkan kembali tombol kategori
                # Aktifkan kembali tombol kategori jika semua tujuan selesai
        if not ordered_categories:
            status_label.config(text="Semua tujuan telah dikunjungi.")
            for btn in kategori_buttons:
                btn.config(state=tk.NORMAL)
    else:
        root.after(500, cek_kedatangan)  # Periksa lagi setelah 500ms

def kembali_ke_awal():
    if robot_commander:
        pose = goal_pose_map["Posisi Awal"]
        robot_commander.send_goal_pose(*pose)
        status_label.config(text="Kembali ke posisi awal...")

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
root.geometry("430x500")
root.resizable(False, False)

ikon_path = "/home/orin/superbot_ws/src/superbot_core/superbot.png"
if os.path.exists(ikon_path):
    try:
        icon = PhotoImage(file=ikon_path)
        root.iconphoto(True, icon)
    except Exception as e:
        print(f"[!] Gagal memuat ikon: {e}")

judul_label = tk.Label(root, text="Navigasi Robot Supermarket",
                       font=("Helvetica", 14, "bold"),
                       bg="#f0f8ff", fg="#2f4f4f")
judul_label.pack(pady=(10, 5))

kategori_frame = tk.Frame(root, bg="#f0f8ff")
kategori_frame.pack(padx=10)

urutan_bagian = [("Bagian A", 0, 0),
                 ("Bagian B", 0, 1),
                 ("Bagian C", 1, 0),
                 ("Bagian D", 1, 1)]

for bagian, row, col in urutan_bagian:
    frame = tk.LabelFrame(kategori_frame, text=bagian,
                          font=("Arial", 10, "bold"),
                          bg="#f0f8ff", fg="#2f4f4f",
                          labelanchor="n", padx=8, pady=4, width=190)
    frame.grid(row=row, column=col, padx=8, pady=8, sticky="n")
    for kategori in bagian_kategori[bagian]:
        btn = tk.Button(frame, text=kategori, width=22,
                        font=("Arial", 9),
                        bg="#4682b4", fg="white", padx=5, pady=3,
                        command=lambda k=kategori: kirim_goal(k))
        btn.pack(pady=2)
        kategori_buttons.append(btn)

listbox_frame = tk.Frame(root, bg="#f0f8ff")
listbox_frame.pack(pady=9, padx=10)

label_daftar = tk.Label(listbox_frame, text="Barang yang ingin dibeli:",
                        font=("Arial", 10, "bold"),
                        bg="#f0f8ff", fg="#2f4f4f", anchor="center", justify="center")
label_daftar.pack()

listbox_goals = tk.Listbox(listbox_frame, width=40, height=8, font=("Arial", 10), justify="center")
listbox_goals.pack()

button_frame = tk.Frame(root, bg="#f0f8ff")
button_frame.pack(pady=(0, 8))

btn_go = tk.Button(button_frame, text="GO", width=18, font=("Arial", 11, "bold"),
                   bg="#32cd32", fg="white", padx=5, pady=5,
                   command=start_navigation)
btn_go.grid(row=0, column=0, padx=5)

btn_kembali = tk.Button(button_frame, text="Kembali ke Awal", width=18, font=("Arial", 11, "bold"),
                        bg="#ff8c00", fg="white", padx=5, pady=5,
                        command=kembali_ke_awal)
btn_kembali.grid(row=0, column=1, padx=5)

status_label = tk.Label(root, text="Klik kategori untuk memulai navigasi.",
                        font=("Arial", 10, "italic"),
                        bg="#f0f8ff", fg="green")
status_label.pack(pady=(0, 8))

def on_closing():
    if rclpy.ok():
        rclpy.shutdown()
    root.destroy()
    sys.exit()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Jalankan GUI
try:
    root.mainloop()
except KeyboardInterrupt:
    on_closing()
