import serial
import matplotlib.pyplot as plt
from collections import deque

ser = serial.Serial('COM3', 115200)  # ←ポート確認して変更

data_len = 100
x = deque(maxlen=data_len)
y = deque(maxlen=data_len)

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(x, y)

while True:
    line_raw = ser.readline().decode().strip()
    
    try:
        vals = line_raw.split(',')
        val = float(vals[0])  # inputだけ表示
        
        y.append(val)
        x.append(len(x))
        
        line.set_xdata(range(len(y)))
        line.set_ydata(y)
        ax.relim()
        ax.autoscale_view()
        
        plt.draw()
        plt.pause(0.01)
        
    except:
        pass