import serial, threading, sys, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === Cấu hình UART ===
PORT = "COM3"       # Đổi thành COM STM32
BAUD = 115200
TIMEOUT = 1

try:
    ser = serial.Serial(PORT, baudrate=BAUD, timeout=TIMEOUT)
    print(f"✅ UART opened on {PORT} (baud={BAUD})")
except Exception as e:
    print(f"❌ Không mở được cổng {PORT}: {e}")
    sys.exit(1)

# === File log ===
log_file = open("feedback_log.csv", "w")
log_file.write("time,setpoint,speed\n")  # header CSV

running = True
start_time = time.time()

# Dữ liệu để vẽ
times, speeds, setpoints = [], [], []

# === Thread đọc dữ liệu UART ===
def read_feedback():
    global running
    while running:
        try:
            raw = ser.readline()
            if raw:
                line = raw.decode(errors="ignore").strip()
                if line.startswith("FB"):
                    # Format: FB,SET,0.300,SPD,0.280,PWM,120
                    parts = line.split(",")
                    if len(parts) == 7:
                        setpoint = float(parts[2])
                        speed    = float(parts[4])
                        t = time.time() - start_time

                        # Ghi log CSV
                        log_file.write(f"{t:.2f},{setpoint:.3f},{speed:.3f}\n")
                        log_file.flush()

                        # Lưu dữ liệu để vẽ
                        times.append(t)
                        setpoints.append(setpoint)
                        speeds.append(speed)

                        # Giữ buffer gọn
                        if len(times) > 500:
                            times.pop(0)
                            setpoints.pop(0)
                            speeds.pop(0)
        except Exception as e:
            print("❌ Lỗi đọc UART:", e)
            break

# === Thread gửi lệnh CMD ===
def send_commands():
    global running
    print("🚀 Nhập setpoint vận tốc (m/s) và góc lái (0..180)")
    while running:
        try:
            spd = float(input("\nSetpoint tốc độ (m/s): "))
            ang = int(input("Góc lái: "))
            cmd = f"CMD,{spd:.2f},{ang}\r\n"
            ser.write(cmd.encode())
            print(f"[TX] {cmd.strip()}")
        except ValueError:
            print("⚠️ Giá trị không hợp lệ!")
        except KeyboardInterrupt:
            break

# === Plot real-time ===
fig, ax1 = plt.subplots()

# Vẽ speed & setpoint
line_speed, = ax1.plot([], [], label="Speed (m/s)", color="blue")
line_sp, = ax1.plot([], [], label="Setpoint (m/s)", color="orange")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Velocity (m/s)")
ax1.grid()
ax1.legend(loc="upper left")

def update(frame):
    line_speed.set_data(times, speeds)
    line_sp.set_data(times, setpoints)

    ax1.relim()
    ax1.autoscale_view()

    return line_speed, line_sp

ani = animation.FuncAnimation(fig, update, interval=200)

# === Main ===
reader = threading.Thread(target=read_feedback, daemon=True)
reader.start()

sender = threading.Thread(target=send_commands, daemon=True)
sender.start()

try:
    plt.show()
except KeyboardInterrupt:
    print("\n👋 Dừng bởi người dùng")
finally:
    running = False
    log_file.close()
    ser.close()
    print("🔌 UART closed. File feedback_log.csv đã được lưu.")
