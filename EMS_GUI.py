import serial
import tkinter as tk
from tkinter import ttk

ser = serial.Serial('COM4', 115200, timeout=1)

velocity = 0.0
distance = 0.0
peakV = 0.0
texting = "Processing..."
vector = [0.0, 0.0]
maxSpeed = 1.5      #Km/h


# ---------- Button Functions ----------
def readV():
    ser.write(("READ_VELOC" + "\n").encode())
    d_btn.config(state=tk.DISABLED)
    semaphore_btn.config(state=tk.DISABLED)
    radar_btn.config(state=tk.DISABLED)
    peak_btn.config(state=tk.DISABLED)

def readD():
    ser.write(("READ_DIST" + "\n").encode())
    v_btn.config(state=tk.DISABLED)
    semaphore_btn.config(state=tk.DISABLED)
    radar_btn.config(state=tk.DISABLED)
    peak_btn.config(state=tk.DISABLED)

def writeLed():
    ser.write(("ACTIVATE_LED" + "\n").encode())
    d_btn.config(state=tk.DISABLED)
    v_btn.config(state=tk.DISABLED)
    radar_btn.config(state=tk.DISABLED)
    peak_btn.config(state=tk.DISABLED)

def cancelLed():
    global velocity, distance, peakV
    velocity = 0.0
    distance = 0.0
    peakV = 0.0
    v_lb.config(text=f"Velocity: {velocity:.2f} km/h")
    d_lb.config(text=f"Distance: {distance:.2f} cm")
    peak_lb.config(text=f"Peak: {peakV:.2f} V")
    ser.write(("IDLE" + "\n").encode())
    d_btn.config(state=tk.ACTIVE)
    v_btn.config(state=tk.ACTIVE)
    semaphore_btn.config(state=tk.ACTIVE)
    radar_btn.config(state=tk.ACTIVE)
    peak_btn.config(state=tk.ACTIVE)


def radar():
    d_btn.config(state=tk.DISABLED)
    v_btn.config(state=tk.DISABLED)
    semaphore_btn.config(state=tk.DISABLED)
    peak_btn.config(state=tk.DISABLED)
    openRadar()


def go():
    ser.write(("RADAR" + "\n").encode())
    

def peak():
    ser.write(("PEAK" + "\n").encode())
    d_btn.config(state=tk.DISABLED)
    v_btn.config(state=tk.DISABLED)
    semaphore_btn.config(state=tk.DISABLED)
    radar_btn.config(state=tk.DISABLED)




# ---------- Auxiliar Functions ----------
def distanceManegement(value):
    vector[0] = vector[1]
    vector[1] = value

def updateDistance(t, son):
    if(vector[0] > vector[1]):
        t.config(text="Unidentified Object Going BackWards")
    elif(vector[0] < vector[1]):
        t.config(text="Unidentified Object Going Fowards")
    elif(vector[0] == vector[1] and distance != 0):
        t.config(text="Unidentified Object Is Not Moving")
    son.after(100, lambda: updateDistance(t, son))



# ---------- GUI ----------
root = tk.Tk()
root.title("Smart Radar–Ultrasonic Sensor Fusion System for Motion and Distance Profiling")
root.geometry("400x400")

main = ttk.Frame(root, padding=15)
main.grid(row=0, column=0, sticky="nsew")

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# ---------- Labels ----------
v_lb = ttk.Label(
    main,
    text=f"Velocity: {velocity:.2f} km/h",
    font=("Arial", 12),
    anchor="center"
)

d_lb = ttk.Label(
    main,
    text=f"Distance: {distance:.2f} cm",
    font=("Arial", 12),
    anchor="center"
)

peak_lb = ttk.Label(
    main,
    text=f"Peak: {peakV:.2f} V",
    font=("Arial", 12),
    anchor="center"
)



v_lb.grid(row=0, column=0, columnspan=2, pady=10, sticky="ew")
d_lb.grid(row=1, column=0, columnspan=2, pady=10, sticky="ew")
peak_lb.grid(row=2, column=0, columnspan=2, pady=10, sticky="ew")

# ---------- Botões ----------
v_btn = ttk.Button(main, text="Read Velocity", command=readV)
d_btn = ttk.Button(main, text="Read Distance", command=readD)
semaphore_btn = ttk.Button(main, text="Activate Semaphore Mode", command=writeLed)
cancel_btn = ttk.Button(main, text="Cancel", command=cancelLed)
radar_btn = ttk.Button(main, text="Radar", command=radar)
peak_btn = ttk.Button(main, text="Get Peak", command=peak)


v_btn.grid(row=3, column=0, padx=10, pady=15, sticky="ew")
d_btn.grid(row=3, column=1, padx=10, pady=15, sticky="ew")
semaphore_btn.grid(row=4, column=0,padx=10, pady=15,sticky="ew")
peak_btn.grid(row=4, column=1,padx=10, pady=15,sticky="ew")
radar_btn.grid(row=5, column=0,padx=10, pady=15,sticky="ew")
cancel_btn.grid(row=5, column=1,padx=10, pady=15,sticky="ew")


main.columnconfigure(0, weight=1)
main.columnconfigure(1, weight=1)


# ---------- Leitura do ESP32 ----------
def readSerial():
    global velocity, distance, peak

    if ser.in_waiting:
        line = ser.readline().decode().strip()

        if line.startswith("VEL="):
            velocity = float(line.split("=")[1])
            v_lb.config(text=f"Velocity: {velocity:.2f} km/h")

        elif line.startswith("DIST="):
            distance = float(line.split("=")[1])
            d_lb.config(text=f"Distance: {distance:.2f} cm")
            distanceManegement(distance)

        elif line.startswith("PEAK="):
            peakV = float(line.split("=")[1])
            peak_lb.config(text=f"Peak: {peakV:.2f} V")
            
    root.after(100, readSerial)  # chama outra vez daqui a 100 ms


# ---------- Janela do radar ----------
def openRadar():
    nova = tk.Toplevel(root)
    nova.title("Radar")
    nova.geometry("300x300")
    nova.columnconfigure(0, weight=1)
    nova.columnconfigure(1, weight=1)

    close_btn = ttk.Button(nova, text="Close", command=lambda:closeWindow(nova))
    close_btn.grid(row=2, column=1, padx=10, pady=15, sticky="ew")

    radar_lb = ttk.Label(nova, text=texting, font=("Arial", 12), anchor="center")
    radar_lb.grid(row=0, column=0, columnspan=2, pady=10, sticky="ew")

    go_btn = ttk.Button(nova, text="Active", command=lambda:go())
    go_btn.grid(row=2, column=0,padx=10, pady=15, sticky="ew")

    updateDistance(radar_lb, nova)


# ---------- Funções Auxiliares do radar ----------
def closeWindow(n):
    cancelLed()
    n.destroy()


    

readSerial()
root.mainloop()
