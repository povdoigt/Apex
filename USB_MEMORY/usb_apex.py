import serial.tools.list_ports as port_list
import serial
import time

ports = list(port_list.comports())

for p in ports:
    if p.vid == 1155 and p.pid == 22336:
        print(f"APEX found at {p.device}")
        break
else:
    print("APEX not found")
    exit(1)

apex = serial.Serial(p.device, 115200, timeout=1)

file = open("apex_mem.bin", "wb")

t_last = 0

has_received = False
writed_bytes = 0

while True:
    if apex.in_waiting > 0:
        t_last = time.time()

        data = apex.read(apex.in_waiting)
        writed_bytes += file.write(data)
        # apex.reset_input_buffer()

        if writed_bytes % 0x100000 == 0: # 1 Mo
            print(f"Wrote {writed_bytes} bytes ({writed_bytes / 0x100000:.2f} MB)")
    # if writed_bytes >= 0x4000000: # 64 MB
    #     break
    if time.time() - t_last > 1 and writed_bytes > 0:
        print("No data received for 1 seconds, exiting...")
        break

print(f"Wrote {writed_bytes} bytes in total")
file.close()
# Closing the serial port
apex.close()
