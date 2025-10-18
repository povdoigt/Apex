import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import time
import random
import reception_serial as RS



class Data:
    def __init__(self, long, lat, alt, heure, rssi):
        self.long = long
        self.lat = lat
        self.alt = alt
        self.temps = heure
        self.rssi = rssi

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Altitude temps réel")
        self.csv_file = 'com_data.csv'
        self.ser = RS.open_port('COM7', 115200, 1)

        # Données pour le graphe
        self.times = []
        self.altitudes = []

        self.mainframe = ttk.Frame(self)
        self.mainframe.pack(fill='both', expand=True)

        self.fig, self.ax = plt.subplots(figsize=(6,4))
        self.ax.set_xlabel("Temps (s)")
        self.ax.set_ylabel("Altitude (m)")
        self.line, = self.ax.plot([], [], 'b-')

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.mainframe)
        self.canvas.get_tk_widget().pack(side='left', fill='both', expand=True)

        self.info_frame = ttk.Frame(self.mainframe)
        self.info_frame.pack(side='right', fill='y', padx=10, pady=10)

        self.labels = {}
        for field in ['Longitude', 'Latitude', 'Altitude', 'Temps', 'RSSI']:
            label = ttk.Label(self.info_frame, text=f"{field}: --")
            label.pack(anchor='w', pady=5)
            self.labels[field] = label

        self.start_time = time.time()

        # Gestion fermeture fenêtre
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.update_data()

    def update_data(self):
        result = RS.read_port(self.ser, self.csv_file)
        if result is not None:
            try:
                d = Data(*result)
            except Exception as e:
                print(f"Erreur création Data: {e}")
                d = None
        else:
            d = None

        if d:
            current_time = time.time() - self.start_time
            self.times.append(current_time)
            self.altitudes.append(d.alt)

            if len(self.times) > 30:
                self.times.pop(0)
                self.altitudes.pop(0)

            self.line.set_data(self.times, self.altitudes)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw()

            self.labels['Longitude'].config(text=f"Longitude: {d.long}")
            self.labels['Latitude'].config(text=f"Latitude: {d.lat}")
            self.labels['Altitude'].config(text=f"Altitude: {d.alt:.2f} m")
            self.labels['Temps'].config(text=f"Temps: {current_time:.1f} s")
            self.labels['RSSI'].config(text=f"RSSI: {d.rssi} dBm")

        print("update_data called")
        self.after_id = self.after(1000, self.update_data)

    def on_closing(self):
        if hasattr(self, 'after_id'):
            self.after_cancel(self.after_id)
        self.destroy()
if __name__ == "__main__":
    app = App()
    app.mainloop()