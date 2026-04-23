import serial
import matplotlib.pyplot as plt
import math

ser = serial.Serial('COM8', 115200)

l1 = 1.0
l2 = 1.0

plt.ion()
fig, ax = plt.subplots()
running = True


def on_key(event):
    global running
    if event.key in ("q", "escape"):
        running = False


fig.canvas.mpl_connect("key_press_event", on_key)

try:
    while running and plt.fignum_exists(fig.number):
        line = ser.readline().decode().strip()

        try:
            vals = line.split(",")
            if len(vals) < 3:
                continue

            output, t1, t2 = map(float, vals[:3])

            # M5と同じ式
            x1 = l1 * math.sin(t1)
            y1 = -l1 * math.cos(t1)

            x2 = x1 + l2 * math.sin(t2)
            y2 = y1 - l2 * math.cos(t2)

            ax.clear()

            ax.plot([0, x1], [0, y1], '-o')
            ax.plot([x1, x2], [y1, y2], '-o')

            ax.set_xlim(-2, 2)
            ax.set_ylim(-2, 2)
            ax.set_aspect('equal')
            ax.set_title("Double Pendulum (q / Esc to quit)")

            # 上下が逆さまになるのを防ぐため、y軸反転はしない
            ax.text(
                0.02,
                0.98,
                f"output: {output:.3f}\ntheta1: {t1:.3f}\ntheta2: {t2:.3f}",
                transform=ax.transAxes,
                va="top",
                ha="left",
                fontsize=10,
                bbox=dict(facecolor="white", alpha=0.8, edgecolor="gray"),
            )

            plt.pause(0.01)

        except (ValueError, UnicodeDecodeError):
            pass
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    plt.close(fig)