import serial
import struct
import threading
import time
import random
import numpy as np

# --- Serial Port Settings ---
SERIAL_PORT = 'COM3'  # Change to your actual COM port
BAUD_RATE = 921600    # Change to match your MCU's UART baud rate
NUM_FLOATS = 10
DATA_SIZE = 4 * NUM_FLOATS
CRC_SIZE = 2
MSG_SIZE = DATA_SIZE + CRC_SIZE
FLOAT_FORMAT = '<' + 'f' * NUM_FLOATS  # Little-endian, 10 floats

# --- Serial Communication Functions ---
def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class MCUComm:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.tx_data = [0.0] * NUM_FLOATS
        self.rx_data = [0.0] * NUM_FLOATS
        self.running = True
        self.send_count = 0
        self.last_send_time = time.time()
        self.min_interval = 0.001  # 1ms target interval
        self.interval_stats = []

    def send(self):
        # Assign random numbers to each element individually
        self.tx_data[0] = 1.24575764878766  # Random number for first value
        self.tx_data[1] = 999  # Random number for second value
        self.tx_data[2] = 7  # Random number for third value
        self.tx_data[3] = 12345.67890  # Random number for fourth value
        self.tx_data[4] = random.uniform(-100.0, 100.0)  # Random number for fifth value
        self.tx_data[5] = random.uniform(-100.0, 100.0)  # Random number for sixth value
        self.tx_data[6] = random.uniform(-100.0, 100.0)  # Random number for seventh value
        self.tx_data[7] = random.uniform(-100.0, 100.0)  # Random number for eighth value
        self.tx_data[8] = random.uniform(-100.0, 100.0)  # Random number for ninth value
        self.tx_data[9] = random.uniform(-100.0, 100.0)  # Random number for tenth value
        
        # Print current values every 100 sends
        if self.send_count % 100 == 0:
            print(f"[TX] Values: {[f'{x:.2f}' for x in self.tx_data]}")
        
        data_bytes = struct.pack(FLOAT_FORMAT, *self.tx_data)
        crc = crc16_ccitt(data_bytes)
        msg = data_bytes + crc.to_bytes(2, 'little')
        self.ser.write(msg)
        
        # Timing statistics
        current_time = time.time()
        interval = current_time - self.last_send_time
        self.interval_stats.append(interval)
        self.last_send_time = current_time
        self.send_count += 1

    def receive(self):
        while self.running:
            try:
                if self.ser.in_waiting >= MSG_SIZE:
                    raw_msg = self.ser.read(MSG_SIZE)
                    data = raw_msg[:DATA_SIZE]
                    crc_recv = int.from_bytes(raw_msg[DATA_SIZE:], 'little')
                    crc_calc = crc16_ccitt(data)
                    if crc_recv == crc_calc:
                        self.rx_data = list(struct.unpack(FLOAT_FORMAT, data))
                        print(f"[RX] {self.rx_data}")
                    else:
                        print(f"[RX] CRC ERROR! Received: {crc_recv:04X}, Calculated: {crc_calc:04X}")
                else:
                    time.sleep(0.001)  # Sleep for 1ms
            except Exception as e:
                print(f"[RX] Error: {e}")
                break

    def print_stats(self):
        if self.interval_stats:
            stats = np.array(self.interval_stats)
            print("\nTiming Statistics:")
            print(f"Total messages sent: {self.send_count}")
            print(f"Average interval: {np.mean(stats)*1000:.3f} ms")
            print(f"Min interval: {np.min(stats)*1000:.3f} ms")
            print(f"Max interval: {np.max(stats)*1000:.3f} ms")
            print(f"Std deviation: {np.std(stats)*1000:.3f} ms")
            print(f"Target interval: {self.min_interval*1000:.3f} ms")

    def close(self):
        self.running = False
        self.print_stats()
        self.ser.close()

def main():
    comm = MCUComm(SERIAL_PORT, BAUD_RATE)
    rx_thread = threading.Thread(target=comm.receive, daemon=True)
    rx_thread.start()
    
    print("Sending random data every 1ms. Press Ctrl+C to stop.")
    try:
        while True:
            comm.send()
            # Calculate sleep time to maintain 1ms interval
            elapsed = time.time() - comm.last_send_time
            sleep_time = max(0, comm.min_interval - elapsed)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        comm.close()

if __name__ == "__main__":
    main() 