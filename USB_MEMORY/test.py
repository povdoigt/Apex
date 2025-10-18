import struct
from dataclasses import dataclass
from matplotlib import pyplot as plt

@dataclass
class Float3:
    x: float
    y: float
    z: float

@dataclass
class Float3Timestamp:
    data: Float3
    timestamp: int

@dataclass
class Float1Timestamp:
    data: float
    timestamp: int

@dataclass
class DataAllTimestamp:
    dummy_start: int
    # adxl375_acc: Float3Timestamp

    bmi088_acc: Float3
    bmi088_gyr: Float3
    # bmp380_temp: Float1Timestamp
    # bmp380_pres: Float1Timestamp
    # lsm303agr_acc: Float3Timestamp
    # lsm303agr_mag: Float3Timestamp
    long: float
    lat: float
    alt: float
    gps_time: float

    timestamp: int
    dummy_end: int

# Format: '<' = little-endian
# f = float (4 bytes), I = uint32 (4 bytes)
STRUCT_FORMAT = '<' + 'H' + 'fff' * 2 + 'ffff' + 'I' + 'H'           # uint32 timestamp

STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

def parse_data_all_timestamp(binary_data):
    unpacked = struct.unpack(STRUCT_FORMAT, binary_data)
    
    idx = 0
    def read_float3():
        nonlocal idx
        f3 = Float3(unpacked[idx], unpacked[idx+1], unpacked[idx+2])
        idx += 3
        return f3

    def read_float1_timestamp():
        nonlocal idx
        val = unpacked[idx]
        ts = unpacked[idx+1]
        idx += 2
        return Float1Timestamp(val, ts)

    def read_float():
        nonlocal idx
        val = float(unpacked[idx])
        idx += 1
        return val

    def read_ts():
        nonlocal idx
        ts = unpacked[idx]
        idx += 1
        return ts
    
    def read_dummy():
        nonlocal idx
        dummy = unpacked[idx]
        idx += 1
        return dummy
    
    ret = DataAllTimestamp(
        dummy_start=read_dummy(),
        # adxl375_acc=read_float3_timestamp(),
        bmi088_acc=read_float3(),
        bmi088_gyr=read_float3(),
        # bmp380_temp=read_float1_timestamp(),
        # bmp380_pres=read_float1_timestamp(),
        # lsm303agr_acc=read_float3_timestamp(),
        # lsm303agr_mag=read_float3_timestamp(),
        long=read_float(),
        lat=read_float(),
        alt=read_float(),
        gps_time=read_float(),

        timestamp=read_ts(),
        dummy_end=read_dummy()
    )

    return ret

nb_entries = 0
nb_good_entries = 0

# Lecture d'un fichier binaire
def read_all_data(file_path):
    global nb_entries, nb_good_entries
    nb_entries = 0

    results = []
    index = 0
    with open(file_path, 'rb') as f:
        while True:
            index += 1
            chunk = f.read(STRUCT_SIZE)
            if len(chunk) < STRUCT_SIZE:
                print("error or end of file at index : " + str(index))
                break
            entry = parse_data_all_timestamp(chunk)
            nb_entries += 1
            if entry.dummy_start != 21930 or entry.dummy_end != 44510:
                # print("Warning: Dummy values bad @ " + str(index))
                # break
                pass
            else:
                results.append(entry)
    return results


bmi088_acc_x = []
bmi088_acc_y = []
bmi088_acc_z = []

bmi088_gyr_x = []
bmi088_gyr_y = []
bmi088_gyr_z = []

long = []
lat = []
alt = []
gps_time = []

time = []

# Example usage
if __name__ == "__main__":
    file_path = "APEX_mem_25\\MROM_11_07_25.bin"
    data_entries = read_all_data(file_path)
    
    print(f"Read {nb_entries} entries, {len(data_entries)} good entries")
    print(f"Percentage of good entries: {len(data_entries) / nb_entries * 100:.2f}%")
    
    first_entry = data_entries[0]
    t0 = first_entry.timestamp
    t0_gps = first_entry.gps_time

    last_timestamp = None
    for entry in data_entries:
        bmi088_acc_x.append(entry.bmi088_acc.x)
        bmi088_acc_y.append(entry.bmi088_acc.y)
        bmi088_acc_z.append(entry.bmi088_acc.z)

        bmi088_gyr_x.append(entry.bmi088_gyr.x)
        bmi088_gyr_y.append(entry.bmi088_gyr.y)
        bmi088_gyr_z.append(entry.bmi088_gyr.z)

        long.append(entry.long)
        lat.append(entry.lat)
        alt.append(entry.alt)
        gps_time.append(entry.gps_time - t0_gps)  # Relative to the first GPS time

        time.append(entry.timestamp - t0) # Timestamp relative to the first entry

        # time.append(entry.bmi088_acc.timestamp)  # Convert to seconds
        # if last_timestamp is None:
        #     last_timestamp = entry.bmi088_acc.timestamp
        #     bmi088_acc_dt.append(0.0)
        # else:
        #     dt = entry.bmi088_acc.timestamp - last_timestamp
        #     bmi088_acc_dt.append(dt)
        #     last_timestamp = entry.bmi088_acc.timestamp

indexs = list(range(len(time)))

plt.figure(figsize=(10, 6))
plt.plot(indexs, bmi088_acc_x, label='bmi088_acc_x')
plt.plot(indexs, bmi088_acc_y, label='bmi088_acc_y')
plt.plot(indexs, bmi088_acc_z, label='bmi088_acc_z')
plt.xlabel('Index')
plt.ylabel('Acceleration (m/s²)')
plt.xlim([0, 400])
plt.title('BMI088 Accelerometer Data')
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(indexs, bmi088_gyr_x, label='bmi088_gyr_x')
plt.plot(indexs, bmi088_gyr_y, label='bmi088_gyr_y')
plt.plot(indexs, bmi088_gyr_z, label='bmi088_gyr_z')
plt.xlabel('Index')
plt.ylabel('Gyroscope (rad/s)')
plt.xlim([0, 400])
plt.ylim([-1000, 1000])
plt.title('BMI088 Gyroscope Data')
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(indexs, long, label='Longitude')
plt.plot(indexs, lat, label='Latitude')
plt.xlabel('Index')
plt.ylabel('GPS Coordinates')
# plt.xlim([0, 400])
plt.title('GPS Data')
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(indexs, alt, label='Altitude')
plt.xlabel('Index')
plt.ylabel('Altitude (m)')
# plt.xlim([0, 400])
plt.title('Altitude Data')
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(indexs, gps_time, label='GPS Time')
plt.xlabel('Index')
plt.ylabel('GPS Time (s)')
plt.xlim([0, 400])
plt.ylim([0, 200])
plt.title('GPS Time Data')
plt.legend()
plt.grid()
plt.show()
