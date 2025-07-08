import serial
import struct
import threading
import time
import random
import numpy as np
import socket

# --- Serial Port Settings ---
SERIAL_PORT = 'COM3'  # Change to your actual COM port
BAUD_RATE = 921600    # Change to match your MCU's UART baud rate

# --- UDP Publishing Settings ---
ENABLE_UDP_PUBLISH = True   # Set to True to publish velocity data via UDP
UDP_IP = "127.0.0.1"        # Localhost
UDP_PORT = 12345            # Port for velocity data publishing

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
        self.consecutive_rx_errors = 0  # Track consecutive CRC errors
        self.max_consecutive_errors = 5  # Clear buffer after 5 consecutive errors
        self.buffer_recoveries = 0      # Count buffer recovery operations
        self.last_send_time = time.time()
        self.min_interval = 0.005  # 5ms target interval to match MCU
        self.interval_stats = []
        
        # MCU error statistics (received from MCU)
        self.mcu_uart_connected = 0      # MCU connection status
        self.mcu_time_since_last_rx = 0  # Time since MCU last received from PC
        self.mcu_crc_errors = 0
        self.mcu_recovery_count = 0
        self.mcu_total_received = 0

        # UDP publishing
        self.udp_socket = None
        if ENABLE_UDP_PUBLISH:
            try:
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print(f"UDP socket created for publishing to {UDP_IP}:{UDP_PORT}")
            except socket.error as e:
                print(f"Error creating UDP socket: {e}")
                print("Disabling UDP publishing...")
                self.udp_socket = None

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
        if self.send_count % 500 == 0:
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
                    
                    if len(raw_msg) == MCU_TO_PC_MSG_SIZE:
                        data = raw_msg[:MCU_TO_PC_DATA_SIZE]
                        crc_recv = int.from_bytes(raw_msg[MCU_TO_PC_DATA_SIZE:], 'little')
                        crc_calc = crc16_ccitt(data)
                        
                        if crc_recv == crc_calc:
                            # *** CRC SUCCESS ***
                            self.consecutive_rx_errors = 0  # Reset consecutive error counter
                            
                            self.rx_data = list(struct.unpack(MCU_TO_PC_FORMAT, data))
                            self.rx_count += 1
                            
                            # Extract MCU error statistics (floats 22, 23, 24)
                            if len(self.rx_data) >= 25:
                                self.mcu_uart_connected = self.rx_data[22]      # 1.0 = connected, 0.0 = disconnected
                                self.mcu_time_since_last_rx = self.rx_data[23]  # Time since MCU last received from PC
                                self.mcu_crc_errors = int(self.rx_data[24])     # Total CRC errors on MCU side
                            
                            # Publish velocity data via UDP
                            if ENABLE_UDP_PUBLISH and self.udp_socket:
                                try:
                                    # Pack velocity data: timestamp + 3 linear vel + 3 angular vel + 3 projected gravity + MCU status
                                    current_time = time.time()
                                    velocity_packet = struct.pack('<d9fi', 
                                        current_time,           # timestamp (double)
                                        self.rx_data[0],        # X linear velocity (float)
                                        self.rx_data[1],        # Y linear velocity (float)  
                                        self.rx_data[2],        # Z linear velocity (float)
                                        self.rx_data[3],        # X angular velocity (float)
                                        self.rx_data[4],        # Y angular velocity (float)
                                        self.rx_data[5],        # Z angular velocity (float)
                                        self.rx_data[6],        # X projected gravity (float)
                                        self.rx_data[7],        # Y projected gravity (float)
                                        self.rx_data[8],        # Z projected gravity (float)
                                        int(self.mcu_uart_connected > 0.5)  # MCU connected (int)
                                    )
                                    self.udp_socket.sendto(velocity_packet, (UDP_IP, UDP_PORT))
                                except socket.error:
                                    # Silently ignore UDP send errors to avoid spam
                                    pass
                            
                            # Print received data every 500 successful receives
                            if self.rx_count % 500 == 0:
                                status_str = "CONNECTED" if self.mcu_uart_connected > 0.5 else "DISCONNECTED"
                                print(f"[RX] Linear Vel: X={self.rx_data[0]:.3f} Y={self.rx_data[1]:.3f} Z={self.rx_data[2]:.3f} m/s")
                                print(f"     Angular Vel: X={self.rx_data[3]:.3f} Y={self.rx_data[4]:.3f} Z={self.rx_data[5]:.3f} rad/s")
                                print(f"     Proj Gravity: X={self.rx_data[6]:.3f} Y={self.rx_data[7]:.3f} Z={self.rx_data[8]:.3f} m/s²")
                                print(f"     MCU UART Status: {status_str} | Last RX: {self.mcu_time_since_last_rx:.0f}ms ago | MCU CRC Errors: {self.mcu_crc_errors}")
                                
                                if self.mcu_uart_connected < 0.5:
                                    print(f"     *** WARNING: MCU reports UART DISCONNECTED! MCU should be beeping! ***")
                                
                                # Print communication statistics
                                success_rate = (self.rx_count / (self.rx_count + self.rx_errors)) * 100 if (self.rx_count + self.rx_errors) > 0 else 0
                                print(f"     PC Stats - TX: {self.send_count} | RX: {self.rx_count} | Errors: {self.rx_errors} | Recoveries: {self.buffer_recoveries} | Success: {success_rate:.1f}%")
                        else:
                            # *** CRC ERROR - Enhanced Error Recovery ***
                            self.rx_errors += 1
                            self.consecutive_rx_errors += 1
                            
                            print(f"[RX] CRC ERROR #{self.rx_errors}! Received: {crc_recv:04X}, Calculated: {crc_calc:04X}")
                            print(f"     Consecutive errors: {self.consecutive_rx_errors}")
                            
                            # Check if too many consecutive errors - perform buffer recovery
                            if self.consecutive_rx_errors >= self.max_consecutive_errors:
                                print(f"     *** BUFFER RECOVERY: Clearing input buffer after {self.consecutive_rx_errors} consecutive errors ***")
                                
                                # Clear the input buffer to resynchronize
                                bytes_cleared = self.ser.in_waiting
                                self.ser.reset_input_buffer()
                                self.buffer_recoveries += 1
                                
                                print(f"     Cleared {bytes_cleared} bytes from buffer (Recovery #{self.buffer_recoveries})")
                                self.consecutive_rx_errors = 0  # Reset counter after recovery
                                
                                # Small delay to let MCU send fresh data
                                time.sleep(0.01)
                            else:
                                # Simple recovery - try to resync by reading one byte
                                print(f"     Simple recovery: Reading 1 byte to resync...")
                                if self.ser.in_waiting > 0:
                                    self.ser.read(1)  # Read and discard 1 byte to shift frame
                            
                            # Print error stats every 10 errors
                            if self.rx_errors % 10 == 0:
                                success_rate = (self.rx_count / (self.rx_count + self.rx_errors)) * 100 if (self.rx_count + self.rx_errors) > 0 else 0
                                print(f"     Error Stats: {self.rx_count} OK, {self.rx_errors} ERR, {self.buffer_recoveries} recoveries, {success_rate:.1f}% success")
                    else:
                        print(f"[RX] Warning: Read {len(raw_msg)} bytes, expected {MCU_TO_PC_MSG_SIZE}")
                        # Clear buffer on incomplete reads
                        self.ser.reset_input_buffer()
                        print("     Cleared input buffer due to incomplete read")
                else:
                    time.sleep(0.001)  # Sleep for 1ms
            except Exception as e:
                print(f"[RX] Error: {e}")
                # Clear buffer on any exception
                self.ser.reset_input_buffer()
                print("     Cleared input buffer due to exception")
                break

    def print_stats(self):
        if self.interval_stats:
            stats = np.array(self.interval_stats)
            print("\nFinal Communication Statistics:")
            print("=" * 50)
            print("PC Side:")
            print(f"  Messages sent to MCU: {self.send_count}")
            print(f"  Messages received from MCU: {self.rx_count}")
            print(f"  PC CRC errors: {self.rx_errors}")
            print(f"  Buffer recovery operations: {self.buffer_recoveries}")
            print(f"  Average interval: {np.mean(stats)*1000:.3f} ms")
            print(f"  Min interval: {np.min(stats)*1000:.3f} ms")
            print(f"  Max interval: {np.max(stats)*1000:.3f} ms")
            print(f"  Target interval: {self.min_interval*1000:.3f} ms")
            
            if (self.rx_count + self.rx_errors) > 0:
                success_rate = (self.rx_count / (self.rx_count + self.rx_errors)) * 100
                print(f"  PC Success rate: {success_rate:.1f}%")
            
            print("\nMCU Side (from MCU transmission):")
            mcu_status = "CONNECTED" if self.mcu_uart_connected > 0.5 else "DISCONNECTED"
            print(f"  MCU UART Status: {mcu_status}")
            print(f"  Time since MCU last received from PC: {self.mcu_time_since_last_rx:.0f} ms")
            print(f"  Total CRC errors reported by MCU: {self.mcu_crc_errors}")
            
            if self.mcu_uart_connected < 0.5:
                print(f"  *** MCU was reporting DISCONNECTED status (buzzer should have been beeping) ***")
            
            if ENABLE_UDP_PUBLISH and self.udp_socket:
                print(f"\nUDP Publishing:")
                print(f"  Socket bound to {UDP_IP}:{UDP_PORT}")

    def close(self):
        self.running = False
        
        # Close UDP socket
        if ENABLE_UDP_PUBLISH and self.udp_socket:
            try:
                self.udp_socket.close()
                print("UDP socket closed successfully")
            except Exception as e:
                print(f"Error closing UDP socket: {e}")
        
        self.print_stats()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main():
    comm = MCUComm(SERIAL_PORT, BAUD_RATE)
    rx_thread = threading.Thread(target=comm.receive, daemon=True)
    rx_thread.start()
    
    print("Asymmetric UART Communication Test with CRC Error Recovery")
    print("=" * 60)
    print(f"PC → MCU: {PC_TO_MCU_FLOATS} floats ({PC_TO_MCU_MSG_SIZE} bytes)")
    print(f"MCU → PC: {MCU_TO_PC_FLOATS} floats ({MCU_TO_PC_MSG_SIZE} bytes)")
    print("=" * 60)
    print("Features:")
    print("  • Automatic CRC error recovery")
    print("  • Timeout-based error detection")
    print("  • Real-time error statistics from both PC and MCU")
    if ENABLE_UDP_PUBLISH:
        print("  • UDP publishing of 9D motion data (linear + angular + gravity) for visualization")
    print("=" * 60)
    print("Press Ctrl+C to stop.")
    if ENABLE_UDP_PUBLISH:
        print("UDP data will be published to {}:{} (9D motion: linear + angular + gravity)".format(UDP_IP, UDP_PORT))
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