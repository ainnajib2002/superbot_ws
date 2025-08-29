from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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

# -------------------------------
# Load Cell Configuration
# -------------------------------
BAUDRATE = 115200
PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0"
MAX_WARNING = 37.0
MAX_LOAD = 40.0

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

# -------------------------------
# Dictionary for goal poses based on categories
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
    "Posisi Awal": (0.0, 0.0, 0.0, 1.0)  # Initial pose
}

bagian_kategori = {
    "Bagian A": ["Makanan", "Minuman"],
    "Bagian B": ["Produk Rumah Tangga", "Kesehatan & Kecantikan"],
    "Bagian C": ["Snack", "Bahan Dapur"],
    "Bagian D": ["Perawatan Bayi", "Elektronik"]
}

# -------------------------------
# RobotCommander Class (ROS2 Node)
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

    def pose_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        if self.goal_pose is None:
            return
        dist = math.sqrt((self.current_pose[0] - self.goal_pose[0])**2 +
                         (self.current_pose[1] - self.goal_pose[1])**2)
        if dist < 0.3:
            self.goal_pose = None
            self.get_logger().info("Robot telah sampai di tujuan.")
            if status_label:
                status_label.after(0, lambda: status_label.config(text="Robot telah sampai di tujuan!"))
                status_label.after(500, cek_kedatangan)

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
            status_label.config(text=f"'{kategori}' ditambahkan ke daftar pembelian.")
        else:
            status_label.config(text=f"'{kategori}' sudah ada di daftar pembelian.")
    else:
        status_label.config(text="Kategori tidak valid.")

# -------------------------------
# Navigate to the First Category
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

# -------------------------------
# Check if Arrived at Destination
# -------------------------------
def cek_kedatangan():
    global navigasi_berjalan, current_kategori, ordered_categories

    if robot_commander.goal_pose is None:
        # Find index based on category name
        for i in range(listbox_goals.size()):
            if listbox_goals.get(i) == current_kategori:
                listbox_goals.itemconfig(i, {'bg': '#90ee90'})
                listbox_goals.delete(i)
                break

        status_label.config(text=f"Robot telah sampai di '{current_kategori}'. Tekan GO untuk lanjut.")

        # Remove from the list
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
root.geometry("600x1000")  # Adjusted height to accommodate LoadcellApp lebar x tinggi
root.resizable(True, True)

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

# -------------------------------
# LoadcellApp Class
# # -------------------------------
class LoadcellApp:
    def __init__(self, root):
        self.root = root
        self.blinking = False
        self.blink_state = True

        # Label judul
        self.title_label = tk.Label(root, text="Data Loadcell", font=("Helvetica", 14, "bold"),  bg="#f0f8ff", fg="#2f4f4f")
        self.title_label.pack(pady=(25, 0))  # Increased the top padding to move it down
        
        # Frame utama untuk grid
        grid_frame = tk.Frame(root, bg="#f0f8ff")
        grid_frame.pack(pady=5)

        # LabelFrame untuk "Berat"
        berat_frame = tk.LabelFrame(grid_frame, text="Berat", font=("Arial", 10, "bold"),
                                   bg="#f0f8ff", fg="#2f4f4f", labelanchor="n", padx=8, pady=4)
        berat_frame.grid(row=0, column=0, padx=8, pady=8, sticky="n")

        self.weight_value_button = tk.Button(
            berat_frame,
            text="-- kg",
            font=("Arial", 11, "bold"),  # Samakan font dengan tombol reset
            width=20,                    # Samakan lebar dengan tombol reset
            height=1,                    # Samakan tinggi dengan tombol reset
            bg="#f0f8ff",
            fg="black",
            relief="solid",
            bd=2,
            anchor="center",
            justify="center"
        )
        self.weight_value_button.pack(padx=10, pady=1)

        # LabelFrame untuk "Status"
        status_frame = tk.LabelFrame(grid_frame, text="Status", font=("Arial", 10, "bold"),
                                    bg="#f0f8ff", fg="#2f4f4f", labelanchor="n", padx=8, pady=4)
        status_frame.grid(row=0, column=1, padx=8, pady=8, sticky="n")

        self.status_value_button = tk.Button(
            status_frame,
            text="--",
            font=("Arial", 11, "bold"),  # Samakan font dengan tombol reset
            width=20,                    # Samakan lebar dengan tombol reset
            height=1,                    # Samakan tinggi dengan tombol reset
            bg="gray",
            fg="white",
            relief="solid",
            bd=2,
            anchor="center",
            justify="center",
            wraplength=300
        )
        self.status_value_button.pack(padx=10, pady=1)

        # Frame untuk tombol reset dan kalibrasi agar sejajar
        button_frame = tk.Frame(root, bg="#f0f8ff")
        button_frame.pack(pady=5)

        self.reset_button = tk.Button(
                button_frame,
                text="Reset",
                font=("Arial", 11, "bold"), # Samakan font dengan tombol GO
                width=18,                   # Samakan lebar dengan tombol GO
                height=1,                   # Samakan tinggi dengan tombol GO
                bg="#4682b4",             # Biru seperti kategori
                fg="white",
                command=self.reset_status,
                anchor="center",         # Teks di tengah tombol
                justify="center"
            )
        self.reset_button.grid(row=0, column=0, padx=10, pady=2, sticky="nsew")

        # self.calibrate_button = tk.Button(button_frame, text="Kalibrasi", font=("Helvetica", 14, "bold"),
        #                                   width=18, height=2, command=self.auto_calibrate)
        # self.calibrate_button.grid(row=0, column=1, padx=10)

        self.warning_label = tk.Label(root, text="", font=("Helvetica", 14), bg="#f0f8ff", fg="green")
        self.warning_label.pack(pady=5)

        self.running = True
        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.send_command('l')
        except serial.SerialException:
            self.ser = None
            # Tampilkan pesan di terminal, bukan di GUI
            print("[INFO] : ESP32 Belum Connect")  # <--- ini kunci!
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
            except ValueError:
                self.update_weight(None)
            time.sleep(0.1)

    def update_weight(self, berat):
        if berat is None:
            self.root.after(0, lambda: self.weight_value_button.config(text="Data tidak valid", fg="black", bg="#f0f8ff"))
            self.root.after(0, lambda: self.update_status("Tidak Valid", "gray"))
            self.blinking = False
        else:
            self.root.after(0, lambda: self.weight_value_button.config(text=f"{berat:.2f} kg"))
            if berat > MAX_LOAD:
                self.root.after(0, lambda: self.weight_value_button.config(fg="white", bg="red"))
                self.root.after(0, lambda: self.update_status("Melebihi Kapasitas Beban", "red"))
                self.blinking = False
            elif berat > MAX_WARNING:
                self.root.after(0, lambda: self.weight_value_button.config(fg="black", bg="orange"))
                self.root.after(0, lambda: self.update_status("Warning: Hampir Melebihi Batas", "orange"))
                if not self.blinking:
                    self.blinking = True
                    self.blink_warning()
            else:
                self.root.after(0, lambda: self.weight_value_button.config(fg="white", bg="green"))
                self.root.after(0, lambda: self.update_status("Aman", "green"))
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

    # def auto_calibrate(self):
    #     input_berat = simpledialog.askfloat("Kalibrasi", "Masukkan berat aktual dalam kg:")
    #     if input_berat is None or input_berat <= 0:
    #         messagebox.showerror("Error", "Berat tidak valid")
    #         return
    #     raw = self.send_command('g')
    #     print(f"DEBUG: response dari 'g': {repr(raw)}")
    #     try:
    #         raw_val = float(raw)
    #         print(f"DEBUG: raw_val dari 'g': {raw_val}")
    #         if abs(raw_val) < 1:
    #             raise ValueError("Nilai mentah terlalu kecil")
    #         faktor = raw_val / input_berat
    #         self.send_command(f'k {faktor:.2f}')
    #         messagebox.showinfo("Sukses", f"Kalibrasi berhasil!\nFaktor: {faktor:.2f}")
    #     except Exception as e:
    #         messagebox.showerror("Gagal", f"Gagal kalibrasi otomatis: {e}")

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.ser.close()
        self.root.destroy()

# Instantiate LoadcellApp
loadcell_app = LoadcellApp(root)

def on_closing():
    if rclpy.ok():
        rclpy.shutdown()
    root.destroy()
    sys.exit()

root.protocol("WM_DELETE_WINDOW", on_closing)
# Run the GUI
try:
    root.mainloop()
except KeyboardInterrupt:
    on_closing()
