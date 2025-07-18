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

# --- UDP Action Receiving Settings ---
ACTION_UDP_PORT = 12346     # Port for receiving action commands from run_onnx_policy.py
ENABLE_ACTION_RECEIVE = True # Set to True to receive action commands via UDP

# --- UDP Motor Data Publishing Settings ---
MOTOR_UDP_PORT = 12347      # Port for publishing motor data
ENABLE_MOTOR_UDP_PUBLISH = True  # Set to True to publish motor data via UDP

# Round-trip time measurement
PING_SEQUENCE_OFFSET = 6  # Index in tx_data to store ping sequence number
RTT_RESPONSE_OFFSET = 21  # Index in rx_data where MCU echoes back the ping sequence

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
        self.min_interval = 0.02  # 20ms target interval for faster response (50Hz)
        self.interval_stats = []
        
        # Round-trip time measurement
        self.ping_sequence = 0
        self.ping_timestamps = {}  # Dictionary to store timestamps for each ping sequence
        self.rtt_measurements = []  # List to store round-trip time measurements
        self.rtt_min = float('inf')
        self.rtt_max = 0
        self.rtt_avg = 0
        self.mcu_round_trip_time = 0  # Round-trip time measured by MCU
        
        # MCU error statistics (received from MCU)
        self.mcu_uart_connected = 0      # MCU connection status
        self.mcu_time_since_last_rx = 0  # Time since MCU last received from PC
        self.mcu_crc_errors = 0
        self.mcu_recovery_count = 0
        self.mcu_total_received = 0

        # Motor data from MCU (positions and velocities)
        self.motor_positions = [0.0] * 6  # Motor positions [0-5] from rx_data[9-14]
        self.motor_velocities = [0.0] * 6  # Motor velocities [0-5] from rx_data[15-20]

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

        # UDP motor data publishing
        self.motor_udp_socket = None
        if ENABLE_MOTOR_UDP_PUBLISH:
            try:
                self.motor_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print(f"Motor UDP socket created for publishing to {UDP_IP}:{MOTOR_UDP_PORT}")
            except socket.error as e:
                print(f"Error creating motor UDP socket: {e}")
                print("Disabling motor UDP publishing...")
                self.motor_udp_socket = None

        # UDP action receiving
        self.action_udp_socket = None
        # Initialize received_actions with custom values
        self.received_actions = [0.2164, 0.83, -0.785, -0.2164, -0.83, 0.785, 0]  # Store received actions with initial values
        self.action_receive_count = 0
        self.last_action_time = time.time()
        self.action_drop_count = 0  # Track dropped packets
        self.last_counter_received = -1  # Track counter progression
        if ENABLE_ACTION_RECEIVE:
            try:
                self.action_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # Increase receive buffer size to handle bursts
                self.action_udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
                self.action_udp_socket.settimeout(0.0001)  # Ultra-short timeout for minimal latency
                self.action_udp_socket.bind((UDP_IP, ACTION_UDP_PORT))
                print(f"Action UDP socket listening on {UDP_IP}:{ACTION_UDP_PORT} (64KB buffer)")
            except socket.error as e:
                print(f"Error creating action UDP socket: {e}")
                print("Disabling action receiving...")
                self.action_udp_socket = None

    def receive_actions(self):
        """Receive action commands via UDP (non-blocking) - FLUSH BUFFER FOR LATEST"""
        if not self.action_udp_socket:
            return
        
        latest_data = None
        packets_flushed = 0
        
        # FLUSH UDP BUFFER - only keep the latest packet to avoid delay buildup
        try:
            while True:
                data, addr = self.action_udp_socket.recvfrom(1024)
                if len(data) == struct.calcsize('<7f'):
                    latest_data = data
                    packets_flushed += 1
                else:
                    break
        except socket.timeout:
            # Expected - no more packets
            pass
        except Exception as e:
            if self.action_receive_count % 1000 == 0:
                print(f"Action receive error: {e}")
        
        # Process the latest packet only
        if latest_data:
            self.received_actions = list(struct.unpack('<7f', latest_data))
            self.action_receive_count += 1
            self.last_action_time = time.time()
            
            # Track dropped packets and counter progression
            current_counter = int(self.received_actions[6]) if len(self.received_actions) > 6 else 0
            if self.last_counter_received >= 0 and current_counter > self.last_counter_received + 1:
                missed = current_counter - self.last_counter_received - 1
                self.action_drop_count += missed
            self.last_counter_received = current_counter
            
            # Add to drop count for flushed packets (they were late)
            if packets_flushed > 1:
                self.action_drop_count += (packets_flushed - 1)
            
            # Print diagnostics every 200 receives for better monitoring
            if self.action_receive_count % 200 == 0:
                drop_rate = (self.action_drop_count / (self.action_receive_count + self.action_drop_count)) * 100 if (self.action_receive_count + self.action_drop_count) > 0 else 0
                print(f"[ACTION RX] Counter: {current_counter}s | Flushed: {packets_flushed} | Drops: {self.action_drop_count} ({drop_rate:.1f}%)")
                if packets_flushed > 3:
                    print(f"⚠️  WARNING: Flushed {packets_flushed} old packets - buffer overflow detected!")
                    print(f"   Solutions: 1) Reduce send rate 2) Increase processing speed 3) Check system load")

    def send(self):
        # Try to receive new actions
        self.receive_actions()
        
        # Use received actions if available and recent (within 50ms for low latency)
        if (ENABLE_ACTION_RECEIVE and self.action_udp_socket and 
            time.time() - self.last_action_time < 0.05):
            # Use received actions
            self.tx_data = self.received_actions.copy()
        else:
            # Fallback to test values if no recent actions received
            self.tx_data = self.received_actions.copy()
            # self.tx_data[0] = 1.24575764878766  # Test value
            # self.tx_data[1] = 999  # Test value
            # self.tx_data[2] = 7  # Test value
            # self.tx_data[3] = 12345.67890  # Test value
            # self.tx_data[4] = random.uniform(-100.0, 100.0)  # Random value
            # self.tx_data[5] = 12345.67890  # Test value
            # self.tx_data[6] = random.uniform(-100.0, 100.0)  # Random value
        
        # Increment ping sequence number for round-trip time measurement
        self.ping_sequence += 1
        if self.ping_sequence > 65000:
            self.ping_sequence = 1  # Avoid overflow and zero
        
        # Store ping sequence in tx_data for echoing
        self.tx_data[PING_SEQUENCE_OFFSET] = float(self.ping_sequence)
        
        # Record timestamp for this ping sequence
        self.ping_timestamps[self.ping_sequence] = time.time()
        
        # Print current values much less frequently for speed
        if self.send_count % 2000 == 0:
            action_part = [f'{x:.2f}' for x in self.tx_data[:6]]
            counter_part = f"{self.tx_data[6]:.0f}s" if len(self.tx_data) > 6 else "0s"
            print(f"[TX] Actions: {action_part} | Counter: {counter_part}")
        
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
                            
                            # Process round-trip time measurement
                            response_seq = int(self.rx_data[RTT_RESPONSE_OFFSET])
                            if response_seq > 0 and response_seq in self.ping_timestamps:
                                # Calculate round-trip time
                                current_time = time.time()
                                rtt = (current_time - self.ping_timestamps[response_seq]) * 1000  # Convert to ms
                                self.rtt_measurements.append(rtt)
                                
                                # Update min/max/avg
                                self.rtt_min = min(self.rtt_min, rtt)
                                self.rtt_max = max(self.rtt_max, rtt)
                                
                                # Exponential moving average (EMA) with alpha=0.2
                                if len(self.rtt_measurements) == 1:
                                    self.rtt_avg = rtt
                                else:
                                    self.rtt_avg = 0.8 * self.rtt_avg + 0.2 * rtt
                                
                                # Remove old timestamp to avoid memory leak
                                del self.ping_timestamps[response_seq]
                                
                                # Clean up old timestamps (older than 1 second)
                                current_time = time.time()
                                old_keys = [k for k, v in self.ping_timestamps.items() if current_time - v > 1.0]
                                for k in old_keys:
                                    del self.ping_timestamps[k]
                            
                            # Extract MCU error statistics (floats 22, 23, 24)
                            if len(self.rx_data) >= 25:
                                self.mcu_uart_connected = self.rx_data[22]      # 1.0 = connected, 0.0 = disconnected
                                self.mcu_time_since_last_rx = self.rx_data[23]  # Time since MCU last received from PC
                                self.mcu_round_trip_time = self.rx_data[24]     # Round-trip time measured by MCU
                            
                            # Extract motor data (positions and velocities)
                            if len(self.rx_data) >= 21:
                                self.motor_positions = self.rx_data[9:15] # Motor positions [0-5]
                                self.motor_velocities = self.rx_data[15:21] # Motor velocities [0-5]
                            
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
                            
                            # Publish motor data via UDP
                            if ENABLE_MOTOR_UDP_PUBLISH and self.motor_udp_socket:
                                try:
                                    # Pack motor data: timestamp + 6 positions + 6 velocities + MCU status
                                    current_time = time.time()
                                    motor_packet = struct.pack('<d12fi',
                                        current_time,           # timestamp (double)
                                        self.motor_positions[0], # Motor 0 position (float)
                                        self.motor_positions[1], # Motor 1 position (float)
                                        self.motor_positions[2], # Motor 2 position (float)
                                        self.motor_positions[3], # Motor 3 position (float)
                                        self.motor_positions[4], # Motor 4 position (float)
                                        self.motor_positions[5], # Motor 5 position (float)
                                        self.motor_velocities[0], # Motor 0 velocity (float)
                                        self.motor_velocities[1], # Motor 1 velocity (float)
                                        self.motor_velocities[2], # Motor 2 velocity (float)
                                        self.motor_velocities[3], # Motor 3 velocity (float)
                                        self.motor_velocities[4], # Motor 4 velocity (float)
                                        self.motor_velocities[5], # Motor 5 velocity (float)
                                        int(self.mcu_uart_connected > 0.5)  # MCU connected (int)
                                    )
                                    self.motor_udp_socket.sendto(motor_packet, (UDP_IP, MOTOR_UDP_PORT))
                                except socket.error:
                                    # Silently ignore UDP send errors to avoid spam
                                    pass
                            
                            # Print received data much less frequently for speed
                            if self.rx_count % 2000 == 0:
                                status_str = "CONNECTED" if self.mcu_uart_connected > 0.5 else "DISCONNECTED"
                                print(f"[RX] Linear Vel: X={self.rx_data[0]:.3f} Y={self.rx_data[1]:.3f} Z={self.rx_data[2]:.3f} m/s")
                                print(f"     Angular Vel: X={self.rx_data[3]:.3f} Y={self.rx_data[4]:.3f} Z={self.rx_data[5]:.3f} rad/s")
                                print(f"     Proj Gravity: X={self.rx_data[6]:.3f} Y={self.rx_data[7]:.3f} Z={self.rx_data[8]:.3f} m/s²")
                                print(f"     Motor Pos: [{', '.join([f'{pos:.3f}' for pos in self.motor_positions])}] rad")
                                print(f"     Motor Vel: [{', '.join([f'{vel:.3f}' for vel in self.motor_velocities])}] rad/s")
                                print(f"     MCU UART Status: {status_str} | Last RX: {self.mcu_time_since_last_rx:.0f}ms ago")
                                print(f"     Round-trip time: PC={self.rtt_avg:.2f}ms (min={self.rtt_min:.2f}, max={self.rtt_max:.2f}) | MCU={self.mcu_round_trip_time:.2f}ms")
                                
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
                                
                                # No delay - keep communication fast
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
                    time.sleep(0.0001)  # Sleep for 0.1ms - much faster polling
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
            
            print("\nRound-trip Time Measurements:")
            if self.rtt_measurements:
                rtt_stats = np.array(self.rtt_measurements)
                print(f"  PC measured RTT (avg): {self.rtt_avg:.2f} ms")
                print(f"  PC measured RTT (min): {self.rtt_min:.2f} ms")
                print(f"  PC measured RTT (max): {self.rtt_max:.2f} ms")
                print(f"  PC measured RTT (std): {np.std(rtt_stats):.2f} ms")
                print(f"  MCU measured RTT: {self.mcu_round_trip_time:.2f} ms")
            
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
            
            if ENABLE_ACTION_RECEIVE and self.action_udp_socket:
                print(f"\nAction Receiving:")
                print(f"  Socket bound to {UDP_IP}:{ACTION_UDP_PORT}")
                print(f"  Actions received: {self.action_receive_count}")
                action_age = time.time() - self.last_action_time
                action_status = "RECENT" if action_age < 0.05 else "OLD/NONE"
                print(f"  Last action: {action_age:.1f}s ago ({action_status})")
                if self.action_receive_count > 0:
                    action_values = [f'{x:.3f}' for x in self.received_actions[:6]]
                    current_counter = self.received_actions[6] if len(self.received_actions) > 6 else 0
                    drop_rate = (self.action_drop_count / (self.action_receive_count + self.action_drop_count)) * 100 if (self.action_receive_count + self.action_drop_count) > 0 else 0
                    print(f"  Current actions: {action_values}")
                    print(f"  Latest counter: {current_counter:.0f}s (for latency testing)")
                    print(f"  Buffer stats: {self.action_receive_count} received, {self.action_drop_count} dropped ({drop_rate:.1f}%)")

    def close(self):
        self.running = False
        
        # Close UDP sockets
        if ENABLE_UDP_PUBLISH and self.udp_socket:
            try:
                self.udp_socket.close()
                print("UDP publish socket closed successfully")
            except Exception as e:
                print(f"Error closing UDP publish socket: {e}")
        
        if ENABLE_MOTOR_UDP_PUBLISH and self.motor_udp_socket:
            try:
                self.motor_udp_socket.close()
                print("Motor UDP publish socket closed successfully")
            except Exception as e:
                print(f"Error closing motor UDP publish socket: {e}")
        
        if ENABLE_ACTION_RECEIVE and self.action_udp_socket:
            try:
                self.action_udp_socket.close()
                print("UDP action socket closed successfully")
            except Exception as e:
                print(f"Error closing UDP action socket: {e}")
        
        self.print_stats()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main():
    comm = MCUComm(SERIAL_PORT, BAUD_RATE)
    rx_thread = threading.Thread(target=comm.receive, daemon=True)
    rx_thread.start()
    
    print("HIGH-SPEED UART Communication Test with CRC Error Recovery")
    print("=" * 60)
    print(f"PC → MCU: {PC_TO_MCU_FLOATS} floats ({PC_TO_MCU_MSG_SIZE} bytes)")
    print(f"MCU → PC: {MCU_TO_PC_FLOATS} floats ({MCU_TO_PC_MSG_SIZE} bytes)")
    print("=" * 60)
    print("Features:")
    print("  • HIGH-SPEED optimized communication (500Hz target)")
    print("  • ANTI-LATENCY UDP buffer flushing (always uses latest packet)")
    print("  • UART round-trip time measurement (ping-pong echoing)")
    print("  • Automatic CRC error recovery")
    print("  • Timeout-based error detection")
    print("  • Real-time error statistics from both PC and MCU")
    if ENABLE_UDP_PUBLISH:
        print("  • UDP publishing of 9D motion data (linear + angular + gravity) for visualization")
    if ENABLE_MOTOR_UDP_PUBLISH:
        print("  • UDP publishing of motor data (6 positions + 6 velocities) for live graphs")
    if ENABLE_ACTION_RECEIVE:
        print("  • UDP receiving with buffer overflow protection (64KB buffer + flush)")
    print("=" * 60)
    print("Press Ctrl+C to stop.")
    if ENABLE_UDP_PUBLISH:
        print("UDP data will be published to {}:{} (9D motion: linear + angular + gravity)".format(UDP_IP, UDP_PORT))
    if ENABLE_MOTOR_UDP_PUBLISH:
        print("Motor data will be published to {}:{} (6 positions + 6 velocities)".format(UDP_IP, MOTOR_UDP_PORT))
    if ENABLE_ACTION_RECEIVE:
        print("UDP actions will be received on {}:{} (6 joint actions + 1 second counter)".format(UDP_IP, ACTION_UDP_PORT))
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