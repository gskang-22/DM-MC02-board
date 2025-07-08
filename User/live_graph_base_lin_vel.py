import serial
import struct
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# --- Serial Port Settings ---
SERIAL_PORT = 'COM3'  # Change to your actual COM port
BAUD_RATE = 921600

# --- Data Settings ---
MCU_TO_PC_FLOATS = 25
MCU_TO_PC_DATA_SIZE = 4 * MCU_TO_PC_FLOATS
MCU_TO_PC_MSG_SIZE = MCU_TO_PC_DATA_SIZE + 2
MCU_TO_PC_FORMAT = '<' + 'f' * MCU_TO_PC_FLOATS

# --- Graph Settings ---
WINDOW_SIZE = 500  # Number of data points to display
UPDATE_INTERVAL = 20  # Animation update interval in milliseconds

def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    """Calculate CRC16-CCITT for data validation"""
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

class LiveGrapher:
    def __init__(self, port, baud):
        # Serial communication
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.running = True
        
        # Data storage for live graph
        self.time_data = deque(maxlen=WINDOW_SIZE)
        self.vel_x_data = deque(maxlen=WINDOW_SIZE)
        self.vel_y_data = deque(maxlen=WINDOW_SIZE)
        self.vel_z_data = deque(maxlen=WINDOW_SIZE)
        
        # Current values
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.current_vel_z = 0.0
        self.start_time = time.time()
        
        # Statistics
        self.rx_count = 0
        self.rx_errors = 0
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        self.buffer_recoveries = 0
        
        # MCU status
        self.mcu_uart_connected = 0
        self.mcu_time_since_last_rx = 0
        self.mcu_crc_errors = 0
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Setup the plot
        self.setup_plot()
        
        # Start receiving thread
        self.rx_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.rx_thread.start()
    
    def setup_plot(self):
        """Setup the matplotlib figure and axes"""
        plt.style.use('dark_background')  # Dark theme for better visibility
        
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 8))
        self.fig.suptitle('MCU Base Linear Velocity (Real-time)', fontsize=16, color='white')
        
        # Setup each subplot
        colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']  # Red, Teal, Blue
        labels = ['X-Velocity', 'Y-Velocity', 'Z-Velocity']
        units = 'm/s'
        
        self.lines = []
        for i, (ax, color, label) in enumerate(zip(self.axes, colors, labels)):
            line, = ax.plot([], [], color=color, linewidth=2, label=label)
            self.lines.append(line)
            
            ax.set_ylabel(f'{label}\n({units})', fontsize=10, color='white')
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, WINDOW_SIZE * UPDATE_INTERVAL / 1000)  # Time in seconds
            ax.set_ylim(-2.0, 2.0)  # Initial velocity range
            ax.legend(loc='upper right', fontsize=9)
            
            # Styling
            ax.tick_params(colors='white', labelsize=8)
            ax.spines['bottom'].set_color('white')
            ax.spines['top'].set_color('white')
            ax.spines['right'].set_color('white')
            ax.spines['left'].set_color('white')
        
        # Only show x-axis label on bottom plot
        self.axes[-1].set_xlabel('Time (seconds)', fontsize=10, color='white')
        
        # Add status text
        self.status_text = self.fig.text(0.02, 0.95, '', fontsize=9, color='lightgreen', 
                                        verticalalignment='top')
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.92, bottom=0.08)
    
    def receive_data(self):
        """Background thread for receiving UART data"""
        print(f"Started receiving data from {SERIAL_PORT} at {BAUD_RATE} baud")
        print("Graph will update in real-time. Close the graph window to stop.")
        
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
                            self.consecutive_errors = 0
                            rx_data = list(struct.unpack(MCU_TO_PC_FORMAT, data))
                            self.rx_count += 1
                            
                            # Extract velocity data and MCU status
                            with self.data_lock:
                                self.current_vel_x = rx_data[0]  # X linear velocity
                                self.current_vel_y = rx_data[1]  # Y linear velocity  
                                self.current_vel_z = rx_data[2]  # Z linear velocity
                                
                                # MCU status information
                                if len(rx_data) >= 25:
                                    self.mcu_uart_connected = rx_data[22]
                                    self.mcu_time_since_last_rx = rx_data[23]
                                    self.mcu_crc_errors = int(rx_data[24])
                                
                                # Add to time series data
                                current_time = time.time() - self.start_time
                                self.time_data.append(current_time)
                                self.vel_x_data.append(self.current_vel_x)
                                self.vel_y_data.append(self.current_vel_y)
                                self.vel_z_data.append(self.current_vel_z)
                            
                            # Print periodic status
                            if self.rx_count % 100 == 0:
                                status = "CONNECTED" if self.mcu_uart_connected > 0.5 else "DISCONNECTED"
                                print(f"[{self.rx_count:4d}] Vel: X={self.current_vel_x:.3f} Y={self.current_vel_y:.3f} Z={self.current_vel_z:.3f} | MCU: {status}")
                        
                        else:
                            # *** CRC ERROR - Recovery ***
                            self.rx_errors += 1
                            self.consecutive_errors += 1
                            
                            if self.consecutive_errors >= self.max_consecutive_errors:
                                # Buffer recovery
                                bytes_cleared = self.ser.in_waiting
                                self.ser.reset_input_buffer()
                                self.buffer_recoveries += 1
                                self.consecutive_errors = 0
                                print(f"Buffer recovery #{self.buffer_recoveries}: cleared {bytes_cleared} bytes")
                                time.sleep(0.01)
                            else:
                                # Simple recovery
                                if self.ser.in_waiting > 0:
                                    self.ser.read(1)
                    else:
                        # Incomplete read recovery
                        self.ser.reset_input_buffer()
                else:
                    time.sleep(0.001)  # 1ms delay
                    
            except Exception as e:
                print(f"RX Error: {e}")
                self.ser.reset_input_buffer()
                break
    
    def update_plot(self, frame):
        """Animation update function"""
        with self.data_lock:
            if len(self.time_data) < 2:
                return self.lines
            
            # Convert to numpy arrays for plotting
            times = np.array(self.time_data)
            vels = [np.array(self.vel_x_data), 
                    np.array(self.vel_y_data), 
                    np.array(self.vel_z_data)]
            
            # Update each line
            for i, (line, vel_data) in enumerate(zip(self.lines, vels)):
                line.set_data(times, vel_data)
                
                # Auto-scale Y axis based on data range
                if len(vel_data) > 0:
                    data_min, data_max = np.min(vel_data), np.max(vel_data)
                    margin = 0.1 * max(abs(data_min), abs(data_max), 0.1)
                    self.axes[i].set_ylim(data_min - margin, data_max + margin)
            
            # Update X axis to show recent data
            if len(times) > 0:
                time_span = 10  # Show last 10 seconds
                x_max = times[-1]
                x_min = max(0, x_max - time_span)
                for ax in self.axes:
                    ax.set_xlim(x_min, x_max)
            
            # Update status text
            if self.rx_count > 0:
                success_rate = (self.rx_count / (self.rx_count + self.rx_errors)) * 100
                mcu_status = "CONN" if self.mcu_uart_connected > 0.5 else "DISC"
                status_text = (f"RX: {self.rx_count} | Errors: {self.rx_errors} | Success: {success_rate:.1f}% | "
                              f"MCU: {mcu_status} | Recoveries: {self.buffer_recoveries}")
                self.status_text.set_text(status_text)
        
        return self.lines
    
    def start_animation(self):
        """Start the live animation"""
        self.anim = animation.FuncAnimation(
            self.fig, self.update_plot, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.close()
    
    def close(self):
        """Clean shutdown"""
        self.running = False
        if hasattr(self, 'anim'):
            self.anim.event_source.stop()
        if self.ser.is_open:
            self.ser.close()
        
        # Print final statistics
        print("\nFinal Statistics:")
        print(f"  Messages received: {self.rx_count}")
        print(f"  CRC errors: {self.rx_errors}")
        print(f"  Buffer recoveries: {self.buffer_recoveries}")
        if (self.rx_count + self.rx_errors) > 0:
            success_rate = (self.rx_count / (self.rx_count + self.rx_errors)) * 100
            print(f"  Success rate: {success_rate:.1f}%")

def main():
    print("MCU Base Linear Velocity Live Graph")
    print("=" * 40)
    print("This will display real-time X, Y, Z linear velocity data from MCU")
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    print("Close the graph window to stop.")
    print()
    
    try:
        grapher = LiveGrapher(SERIAL_PORT, BAUD_RATE)
        grapher.start_animation()
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("Please check the COM port and ensure the MCU is connected.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 