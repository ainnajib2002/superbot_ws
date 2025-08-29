import serial
import time
import threading
import tkinter as tk
from tkinter import simpledialog, messagebox

# PORT = 'COM3'
BAUDRATE = 115200
PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0"
MAX_WARNING = 37.0
MAX_LOAD = 40.0

class LoadcellApp:
    def __init__(self, root):
        self.root = root
        self.root.title("DATA LOADCELL")
        self.root.geometry("600x450")
        self.root.configure(bg="white")

        self.blinking = False
        self.blink_state = True

        self.title_label = tk.Label(root, text="DATA LOADCELL", font=("Helvetica", 24, "bold"), bg="white")
        self.title_label.pack(pady=10)

        grid_frame = tk.Frame(root, bg="white")
        grid_frame.pack(pady=10)

        self.label_berat = tk.Label(grid_frame, text="Berat", font=("Helvetica", 16, "bold"), bg="white")
        self.label_berat.grid(row=0, column=0, padx=20, pady=10, sticky="w")

        self.label_status = tk.Label(grid_frame, text="Status", font=("Helvetica", 16, "bold"), bg="white")
        self.label_status.grid(row=1, column=0, padx=20, pady=10, sticky="w")

        self.weight_value_button = tk.Button(
            grid_frame, text="-- kg", font=("Helvetica", 20, "bold"),
            bg="white", fg="black", width=16, height=2, relief="solid", bd=2
        )
        self.weight_value_button.grid(row=0, column=1, padx=20, pady=10)

        self.status_value_button = tk.Button(
            grid_frame, text="--", font=("Helvetica", 18, "bold"), width=22, height=2,
            bg="gray", fg="white", wraplength=300, justify="center", relief="solid", bd=2
        )
        self.status_value_button.grid(row=1, column=1, padx=20, pady=10)

        self.reset_button = tk.Button(root, text="Reset", font=("Helvetica", 14, "bold"), command=self.reset_status)
        self.reset_button.pack(pady=5)

        self.calibrate_button = tk.Button(root, text="Kalibrasi", font=("Helvetica", 14, "bold"), command=self.auto_calibrate)
        self.calibrate_button.pack(pady=5)

        self.warning_label = tk.Label(root, text="", font=("Helvetica", 14), bg="white", fg="green")
        self.warning_label.pack(pady=5)

        self.running = True
        self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.send_command('l')

        self.thread = threading.Thread(target=self.read_weight_loop, daemon=True)
        self.thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.close)

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
            self.root.after(0, lambda: self.weight_value_button.config(text="Data tidak valid", fg="black", bg="white"))
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
        self.weight_value_button.config(text="-- kg", bg="white", fg="black")
        self.status_value_button.config(text="--", bg="gray", fg="white")
        self.blinking = False
        try:
            self.send_command('t')
        except Exception as e:
            print("Gagal kirim perintah reset:", e)

    def auto_calibrate(self):
        input_berat = simpledialog.askfloat("Kalibrasi", "Masukkan berat aktual dalam kg:")
        if input_berat is None or input_berat <= 0:
            messagebox.showerror("Error", "Berat tidak valid")
            return
        raw = self.send_command('g')
        print(f"DEBUG: response dari 'g': {repr(raw)}")
        try:
            raw_val = float(raw)
            print(f"DEBUG: raw_val dari 'g': {raw_val}")
            if abs(raw_val) < 1:
                raise ValueError("Nilai mentah terlalu kecil")
            faktor = raw_val / input_berat
            self.send_command(f'k {faktor:.2f}')
            messagebox.showinfo("Sukses", f"Kalibrasi berhasil!\nFaktor: {faktor:.2f}")
        except Exception as e:
            messagebox.showerror("Gagal", f"Gagal kalibrasi otomatis: {e}")

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.ser.close()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = LoadcellApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
