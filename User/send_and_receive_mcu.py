import serial
import struct
import threading
import time
import random
import numpy as np

# --- Serial Port Settings ---
SERIAL_PORT = 'COM3'  # Change to your actual COM port
BAUD_RATE = 921600    # Change to match your MCU's UART baud rate

# PC sends 7 floats to MCU
PC_TO_MCU_FLOATS = 7
PC_TO_MCU_DATA_SIZE = 4 * PC_TO_MCU_FLOATS
PC_TO_MCU_MSG_SIZE = PC_TO_MCU_DATA_SIZE + 2  # 7 floats + 2 bytes CRC
PC_TO_MCU_FORMAT = '<' + 'f' * PC_TO_MCU_FLOATS  # Little-endian, 7 floats

# MCU sends 25 floats to PC
MCU_TO_PC_FLOATS = 25
MCU_TO_PC_DATA_SIZE = 4 * MCU_TO_PC_FLOATS
MCU_TO_PC_MSG_SIZE = MCU_TO_PC_DATA_SIZE + 2  # 25 floats + 2 bytes CRC
MCU_TO_PC_FORMAT = '<' + 'f' * MCU_TO_PC_FLOATS  # Little-endian, 25 floats

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
        self.tx_data = [0.0] * PC_TO_MCU_FLOATS  # PC sends 7 floats
        self.rx_data = [0.0] * MCU_TO_PC_FLOATS  # MCU sends 25 floats
        self.running = True
        self.send_count = 0
        self.rx_count = 0
        self.rx_errors = 0
        self.last_send_time = time.time()
        self.min_interval = 0.005  # 5ms target interval to match MCU
        self.interval_stats = []

    def send(self):
        
        # Add some variation to first few values
        self.tx_data[0] = 1.24575764878766  # Test value
        self.tx_data[1] = 999  # Test value
        self.tx_data[2] = 7  # Test value
        self.tx_data[3] = 12345.67890  # Test value
        self.tx_data[4] = random.uniform(-100.0, 100.0)  # Random value
        self.tx_data[5] = 12345.67890  # Test value
        self.tx_data[6] = random.uniform(-100.0, 100.0)  # Random value
        
        # Print current values every 100 sends
        if self.send_count % 100 == 0:
            print(f"[TX] Sending {PC_TO_MCU_FLOATS} floats: {[f'{x:.2f}' for x in self.tx_data]}")
        
        # Pack and send 7 floats
        data_bytes = struct.pack(PC_TO_MCU_FORMAT, *self.tx_data)
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
                if self.ser.in_waiting >= MCU_TO_PC_MSG_SIZE:
                    raw_msg = self.ser.read(MCU_TO_PC_MSG_SIZE)
                    data = raw_msg[:MCU_TO_PC_DATA_SIZE]
                    crc_recv = int.from_bytes(raw_msg[MCU_TO_PC_DATA_SIZE:], 'little')
                    crc_calc = crc16_ccitt(data)
                    if crc_recv == crc_calc:
                        self.rx_data = list(struct.unpack(MCU_TO_PC_FORMAT, data))
                        self.rx_count += 1
                        
                        # Print received data every 50 successful receives
                        if self.rx_count % 50 == 0:
                            print(f"[RX] Received {MCU_TO_PC_FLOATS} floats: {[f'{x:.2f}' for x in self.rx_data[:5]]}... {self.rx_data[-1]:.2f}")
                            success_rate = (self.rx_count / (self.rx_count + self.rx_errors)) * 100
                            print(f"     TX: {self.send_count} | RX: {self.rx_count} | Errors: {self.rx_errors} | Success: {success_rate:.1f}%")
                    else:
                        self.rx_errors += 1
                        if self.rx_errors % 10 == 0:
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
            print(f"Total messages received: {self.rx_count}")
            print(f"Total CRC errors: {self.rx_errors}")
            print(f"Average interval: {np.mean(stats)*1000:.3f} ms")
            print(f"Min interval: {np.min(stats)*1000:.3f} ms")
            print(f"Max interval: {np.max(stats)*1000:.3f} ms")
            print(f"Std deviation: {np.std(stats)*1000:.3f} ms")
            print(f"Target interval: {self.min_interval*1000:.3f} ms")
            if (self.rx_count + self.rx_errors) > 0:
                success_rate = (self.rx_count / (self.rx_count + self.rx_errors)) * 100
                print(f"Success rate: {success_rate:.1f}%")

    def close(self):
        self.running = False
        self.print_stats()
        self.ser.close()

def main():
    comm = MCUComm(SERIAL_PORT, BAUD_RATE)
    rx_thread = threading.Thread(target=comm.receive, daemon=True)
    rx_thread.start()
    
    print("Asymmetric UART Communication Test")
    print("=" * 50)
    print(f"PC → MCU: {PC_TO_MCU_FLOATS} floats ({PC_TO_MCU_MSG_SIZE} bytes)")
    print(f"MCU → PC: {MCU_TO_PC_FLOATS} floats ({MCU_TO_PC_MSG_SIZE} bytes)")
    print("=" * 50)
    print("Press Ctrl+C to stop.")
    print()
    
    try:
        while True:
            comm.send()
            # Calculate sleep time to maintain 5ms interval
            elapsed = time.time() - comm.last_send_time
            sleep_time = max(0, comm.min_interval - elapsed)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        comm.close()

if __name__ == "__main__":
    main() 