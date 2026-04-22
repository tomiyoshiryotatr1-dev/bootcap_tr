import serial
import matplotlib.pyplot as plt
import math

ser = serial.Serial('COM8', 115200)  # ←ポート変更

l1 = 1.0
l2 = 1.0

plt.ion()
fig, ax = plt.subplots()

while True:
    line = ser.readline().decode().strip()

    try:
        t1, t2 = map(float, line.split(","))

        # 座標計算
        x1 = l1 * math.sin(t1)
        y1 = l1 * math.cos(t1)

        x2 = x1 + l2 * math.sin(t2)
        y2 = y1 + l2 * math.cos(t2)

        ax.clear()

        # 描画
        ax.plot([0, x1], [0, y1], '-o')   # 第1リンク
        ax.plot([x1, x2], [y1, y2], '-o') # 第2リンク

        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')

        plt.pause(0.01)

    except:
        pass