import serial
import csv
import time
import os 


PORT = 'COM7'
BAUDRATE = 115200
TIMEOUT = 1

CSV_FILE = 'com_data.csv'


def open_port(PORT,BAUDRATE,TIMEOUT):
    seri = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    print(f"Connexion au port {PORT} à {BAUDRATE} bauds...")    
    return seri 

def read_port_1(ser,CSV_FILE):
    line = ser.readline().decode('utf-8').strip()
        
    if line:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"{timestamp} - Reçu : {line}")                   
        with open('reception.csv', 'a', newline='') as f:
            writer = csv.writer(f, quoting=csv.QUOTE_MINIMAL)

            # Écrire un en-tête si besoin
            writer.writerow(['Horodatage', 'Donnée'])
            # Écriture dans le CSV
            writer.writerow([timestamp, line])
    data = line.split(',')
    for i,j in enumerate(data):
        data[i] = float(j)
    d_tuple = tuple(data)
    return (d_tuple)


def read_port(ser, CSV_FILE):
    line = ser.readline().decode('utf-8').strip()

    if not line:
        return None  # Rien reçu

    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"{timestamp} - Reçu : {line}")

    # Sauvegarder toute ligne reçue, même texte
    write_header = not os.path.isfile(CSV_FILE)
    with open(CSV_FILE, 'a', newline='') as f:
        writer = csv.writer(f, quoting=csv.QUOTE_MINIMAL, escapechar='\\')
        if write_header:
            writer.writerow(['Horodatage', 'Donnée'])
        writer.writerow([timestamp, line])

    # Essayer de parser la ligne en float
    try:
        data = [float(x) for x in line.split(',')]
        return tuple(data)
    except ValueError:
        print("Erreur de parsing des données : ligne ignorée.")
        return None

        

