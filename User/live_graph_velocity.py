import socket
import struct
import time
import threading
import matplotlib
# Try different backends for better compatibility
try:
    matplotlib.use('TkAgg')  # Try TkAgg first
except ImportError:
    try:
        matplotlib.use('Qt5Agg')  # Fallback to Qt5Agg
    except ImportError:
        matplotlib.use('Agg')  # Final fallback to non-interactive
        print("Warning: Using non-interactive backend")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# --- UDP Settings ---
UDP_IP = "127.0.0.1"        # Localhost
UDP_PORT = 12345            # Port to receive velocity data
SOCKET_TIMEOUT = 0.1        # Socket timeout in seconds

# --- Graph Settings ---
WINDOW_SIZE = 800           # Optimized buffer size for smoothness (was 1000)
TIME_WINDOW = 4.0           # Shorter time window for smoother scrolling (was 5.0)
UPDATE_INTERVAL = 16        # Smoother 60+ FPS animation (was 20ms)
DOWNSAMPLE_FACTOR = 2       # Plot every Nth point for smoother performance

class VelocityGrapher:
    def __init__(self):
        # Data storage for live graph - both linear and angular velocity
        self.time_data = deque(maxlen=WINDOW_SIZE)
        
        # Linear velocity data (rx_data[0-2])
        self.vel_x_data = deque(maxlen=WINDOW_SIZE)
        self.vel_y_data = deque(maxlen=WINDOW_SIZE)
        self.vel_z_data = deque(maxlen=WINDOW_SIZE)
        
        # Angular velocity data (rx_data[3-5])
        self.ang_x_data = deque(maxlen=WINDOW_SIZE)
        self.ang_y_data = deque(maxlen=WINDOW_SIZE)
        self.ang_z_data = deque(maxlen=WINDOW_SIZE)
        
        # Projected gravity data (rx_data[6-8])
        self.grav_x_data = deque(maxlen=WINDOW_SIZE)
        self.grav_y_data = deque(maxlen=WINDOW_SIZE)
        self.grav_z_data = deque(maxlen=WINDOW_SIZE)
        
        # Current values
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.current_vel_z = 0.0
        self.current_ang_x = 0.0
        self.current_ang_y = 0.0
        self.current_ang_z = 0.0
        self.current_grav_x = 0.0
        self.current_grav_y = 0.0
        self.current_grav_z = 0.0
        self.mcu_connected = False
        self.last_update_time = time.time()
        
        # Statistics
        self.packet_count = 0
        self.error_count = 0
        self.start_time = time.time()
        self.last_rate_check = time.time()
        self.packets_at_last_check = 0
        self.current_data_rate = 0.0
        
        # Thread safety
        self.data_lock = threading.Lock()
        self.running = True
        
        # Performance optimization
        self.last_autoscale_time = time.time()
        self.autoscale_interval = 1.5  # Faster auto-scale for smoother adaptation (was 2.0)
        self.frame_count = 0
        self.skip_heavy_operations = 0  # Skip heavy operations occasionally for smoothness
        
        # Setup UDP socket
        self.setup_udp()
        
        # Setup the plot
        self.setup_plot()
        
        # Start receiving thread
        self.rx_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.rx_thread.start()
    
    def setup_udp(self):
        """Setup UDP socket for receiving velocity data"""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.settimeout(SOCKET_TIMEOUT)
            self.udp_socket.bind((UDP_IP, UDP_PORT))
            print(f"UDP socket listening on {UDP_IP}:{UDP_PORT}")
        except socket.error as e:
            print(f"Error setting up UDP socket: {e}")
            print("Make sure send_and_receive_mcu.py is running with ENABLE_UDP_PUBLISH = True")
            exit(1)
    
    def setup_plot(self):
        """Setup the matplotlib figure and axes in 3x3 layout"""
        try:
            plt.ion()  # Enable interactive mode for better real-time updates
        except:
            print("Warning: Could not enable interactive mode")
            
        plt.style.use('dark_background')  # Dark theme for better visibility
        
        try:
            # Try 3x3 grid layout first
            self.fig, self.axes = plt.subplots(3, 3, figsize=(14, 8))  # Reduced from 18x10
            self.fig.suptitle('MCU 9D Motion Data (Real-time via UDP)', fontsize=14, color='white')
            self.use_grid_layout = True
            print("Using 3x3 grid layout")
        except Exception as e:
            print(f"Warning: Could not create 3x3 grid: {e}")
            print("Falling back to simple 3-column layout...")
            # Fallback to simpler layout
            self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 8))
            self.fig.suptitle('MCU Motion Data (Simplified Layout)', fontsize=14, color='white') 
            self.use_grid_layout = False
        
        # Colors for each data type
        linear_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']      # Red, Teal, Blue
        angular_colors = ['#FFD700', '#90EE90', '#FFB6C1']     # Gold, LightGreen, Pink  
        gravity_colors = ['#FFA500', '#32CD32', '#DA70D6']     # Orange, LimeGreen, Orchid
        
        # Labels and units for each column
        axis_labels = ['X', 'Y', 'Z']
        column_titles = ['Linear Velocity (m/s)', 'Angular Velocity (rad/s)', 'Projected Gravity (m/s²)']
        
        self.lines = []
        
        if self.use_grid_layout:
            # Setup 3x3 grid layout
            for row in range(3):  # X, Y, Z
                for col in range(3):  # Linear, Angular, Gravity
                    ax = self.axes[row, col]
                    
                    # Choose colors based on column
                    if col == 0:    # Linear velocity
                        color = linear_colors[row]
                        y_range = (-2.0, 2.0)
                    elif col == 1:  # Angular velocity  
                        color = angular_colors[row]
                        y_range = (-5.0, 5.0)
                    else:           # Projected gravity
                        color = gravity_colors[row]
                        y_range = (-12.0, 12.0)  # Gravity range
                    
                    # Create the line plot
                    line, = ax.plot([], [], color=color, linewidth=2, 
                                   label=f'{axis_labels[row]}-{column_titles[col].split()[0]}')
                    self.lines.append(line)
                    
                    # Configure the subplot
                    ax.set_ylabel(f'{axis_labels[row]}-{column_titles[col].split()[0]}', 
                                 fontsize=9, color='white')
                    ax.grid(True, alpha=0.3)
                    ax.set_xlim(0, 5)  # Initial time window
                    ax.set_ylim(y_range[0], y_range[1])
                    ax.legend(loc='upper right', fontsize=7)
                    
                    # Styling
                    ax.tick_params(colors='white', labelsize=7)
                    for spine in ax.spines.values():
                        spine.set_color('white')
                    
                    # Add column titles to top row
                    if row == 0:
                        ax.set_title(column_titles[col], fontsize=11, color='white', pad=10)
            
            # Only show x-axis labels on bottom row
            for col in range(3):
                self.axes[2, col].set_xlabel('Time (seconds)', fontsize=9, color='white')
                
        else:
            # Setup fallback simple layout (3 rows, 1 column)
            simple_titles = ['Linear Velocity XYZ (m/s)', 'Angular Velocity XYZ (rad/s)', 'Projected Gravity XYZ (m/s²)']
            colors_sets = [linear_colors, angular_colors, gravity_colors]
            y_ranges = [(-2.0, 2.0), (-5.0, 5.0), (-12.0, 12.0)]
            
            for row in range(3):
                ax = self.axes[row]
                colors = colors_sets[row]
                
                # Create 3 lines per subplot (X, Y, Z)
                for i, (axis_label, color) in enumerate(zip(axis_labels, colors)):
                    line, = ax.plot([], [], color=color, linewidth=2, label=f'{axis_label}')
                    self.lines.append(line)
                
                ax.set_title(simple_titles[row], fontsize=11, color='white')
                ax.set_ylabel('Value', fontsize=9, color='white')
                ax.grid(True, alpha=0.3)
                ax.set_xlim(0, 5)
                ax.set_ylim(y_ranges[row][0], y_ranges[row][1])
                ax.legend(loc='upper right', fontsize=8)
                
                # Styling
                ax.tick_params(colors='white', labelsize=8)
                for spine in ax.spines.values():
                    spine.set_color('white')
            
            # Only show x-axis label on bottom plot
            self.axes[-1].set_xlabel('Time (seconds)', fontsize=9, color='white')
        
        # Add status text
        self.status_text = self.fig.text(0.02, 0.98, '', fontsize=8, color='lightgreen', 
                                        verticalalignment='top')
        
        # Optimized layout for better performance
        plt.tight_layout(pad=1.0)  # Reduced padding for faster rendering
        if self.use_grid_layout:
            plt.subplots_adjust(top=0.92, bottom=0.08, hspace=0.25, wspace=0.25)  # Reduced spacing
        else:
            plt.subplots_adjust(top=0.92, bottom=0.08)
    
    def receive_data(self):
        """Background thread for receiving UDP velocity data"""
        print("Started receiving velocity data...")
        print("Close the graph window to stop.")
        
        while self.running:
            try:
                # Receive UDP packet
                data, addr = self.udp_socket.recvfrom(1024)
                
                # Unpack velocity data: timestamp + X, Y, Z velocity + MCU status
                if len(data) == struct.calcsize('<d9fi'): # Changed to 9 floats
                    timestamp, vel_x, vel_y, vel_z, ang_x, ang_y, ang_z, grav_x, grav_y, grav_z, mcu_status = struct.unpack('<d9fi', data)
                    
                    with self.data_lock:
                        # Store relative time from start
                        relative_time = timestamp - self.start_time
                        self.time_data.append(relative_time)
                        self.vel_x_data.append(vel_x)
                        self.vel_y_data.append(vel_y)
                        self.vel_z_data.append(vel_z)
                        self.ang_x_data.append(ang_x)
                        self.ang_y_data.append(ang_y)
                        self.ang_z_data.append(ang_z)
                        self.grav_x_data.append(grav_x)
                        self.grav_y_data.append(grav_y)
                        self.grav_z_data.append(grav_z)
                        
                        # Update current values
                        self.current_vel_x = vel_x
                        self.current_vel_y = vel_y
                        self.current_vel_z = vel_z
                        self.current_ang_x = ang_x
                        self.current_ang_y = ang_y
                        self.current_ang_z = ang_z
                        self.current_grav_x = grav_x
                        self.current_grav_y = grav_y
                        self.current_grav_z = grav_z
                        self.mcu_connected = bool(mcu_status)
                        self.last_update_time = time.time()
                    
                    self.packet_count += 1
                    
                    # Calculate data rate every second
                    current_time = time.time()
                    if current_time - self.last_rate_check >= 1.0:
                        packets_this_interval = self.packet_count - self.packets_at_last_check
                        self.current_data_rate = packets_this_interval / (current_time - self.last_rate_check)
                        self.last_rate_check = current_time
                        self.packets_at_last_check = self.packet_count
                    
                    # Print periodic status with more details (reduced frequency for smoothness)
                    if self.packet_count % 1000 == 0:  # Much less frequent printing (was 500)
                        status = "CONNECTED" if self.mcu_connected else "DISCONNECTED"
                        buffer_usage = len(self.time_data)
                        time_span = relative_time - self.time_data[0] if len(self.time_data) > 1 else 0
                        print(f"[{self.packet_count:4d}] Linear: X={vel_x:.3f} Y={vel_y:.3f} Z={vel_z:.3f} m/s")
                        print(f"         Angular: X={ang_x:.3f} Y={ang_y:.3f} Z={ang_z:.3f} rad/s")
                        print(f"         Gravity: X={grav_x:.3f} Y={grav_y:.3f} Z={grav_z:.3f} m/s²")
                        print(f"         MCU: {status} | Rate: {self.current_data_rate:.1f} Hz | Buffer: {buffer_usage}/{WINDOW_SIZE} | Span: {time_span:.1f}s")
                
                else:
                    self.error_count += 1
                    if self.error_count % 10 == 0:
                        print(f"Invalid packet size: {len(data)} bytes (expected {struct.calcsize('<d9fi')})")
                        
            except socket.timeout:
                # Check if data is too old (no updates for 2 seconds)
                if time.time() - self.last_update_time > 2.0:
                    with self.data_lock:
                        self.mcu_connected = False
                continue
                
            except Exception as e:
                print(f"Error receiving UDP data: {e}")
                break
    
    def update_plot(self, frame):
        """Animation update function - optimized for maximum smoothness"""
        self.frame_count += 1
        self.skip_heavy_operations += 1
        
        with self.data_lock:
            if len(self.time_data) < 2:
                return self.lines
            
            # Convert to numpy arrays for plotting
            times = np.array(self.time_data)
            
            # Organize data in 3x3 grid order: 
            # Row 0: X-Linear, X-Angular, X-Gravity
            # Row 1: Y-Linear, Y-Angular, Y-Gravity  
            # Row 2: Z-Linear, Z-Angular, Z-Gravity
            all_data = [
                np.array(self.vel_x_data),   # [0,0] X-Linear
                np.array(self.ang_x_data),   # [0,1] X-Angular  
                np.array(self.grav_x_data),  # [0,2] X-Gravity
                np.array(self.vel_y_data),   # [1,0] Y-Linear
                np.array(self.ang_y_data),   # [1,1] Y-Angular
                np.array(self.grav_y_data),  # [1,2] Y-Gravity
                np.array(self.vel_z_data),   # [2,0] Z-Linear
                np.array(self.ang_z_data),   # [2,1] Z-Angular
                np.array(self.grav_z_data)   # [2,2] Z-Gravity
            ]
            
            # Optimized downsampling for smoother performance
            if len(times) > 50:  # Start downsampling earlier
                # Dynamic step calculation for consistent performance
                step = max(1, len(times) // 400)  # Reduced max points to 400 for smoothness
                times = times[::step]
                all_data = [data[::step] for data in all_data]
            
            # Update each line - this is the core operation, keep it fast
            for i, (line, data) in enumerate(zip(self.lines, all_data)):
                line.set_data(times, data)
            
            # Update X axis for all subplots (do this every frame for smooth scrolling)
            if len(times) > 0:
                x_max = times[-1]
                x_min = x_max - TIME_WINDOW  # Always show TIME_WINDOW seconds
                
                # Don't go below 0 if we don't have enough data yet
                if x_min < 0:
                    x_min = 0
                
                # Update all subplots with new time range
                if self.use_grid_layout:
                    for row in range(3):
                        for col in range(3):
                            self.axes[row, col].set_xlim(x_min, x_max)
                else:
                    for row in range(3):
                        self.axes[row].set_xlim(x_min, x_max)
            
            # Auto-scale Y axis less frequently but more responsive
            current_time = time.time()
            if (current_time - self.last_autoscale_time > self.autoscale_interval and 
                self.skip_heavy_operations % 3 == 0):  # Skip some auto-scale cycles
                self.last_autoscale_time = current_time
                
                # Auto-scale each subplot
                for i, data in enumerate(all_data):
                    if len(data) > 0:
                        if self.use_grid_layout:
                            row = i // 3
                            col = i % 3
                            ax = self.axes[row, col]
                            
                            data_min, data_max = np.min(data), np.max(data)
                            margin = 0.1 * max(abs(data_min), abs(data_max), 0.1)
                            ax.set_ylim(data_min - margin, data_max + margin)
                        else:
                            # In fallback mode, only update axis limits for whole data types
                            # Skip individual auto-scaling to avoid conflicts
                            pass
            
            # Update status text much less frequently for smoother animation
            if self.frame_count % 30 == 0:  # Update status every 30 frames (~480ms)
                if self.packet_count > 0:
                    connection_status = "CONNECTED" if self.mcu_connected else "DISCONNECTED"
                    data_age = time.time() - self.last_update_time
                    data_span = times[-1] - times[0] if len(times) > 1 else 0
                    buffer_usage = len(self.time_data)
                    status_text = (f"Packets: {self.packet_count} | Rate: {self.current_data_rate:.1f} Hz | "
                                  f"Buffer: {buffer_usage}/{WINDOW_SIZE} | Span: {data_span:.1f}s/{TIME_WINDOW:.0f}s | MCU: {connection_status}")
                    self.status_text.set_text(status_text)
        
        # Reduce forced redraw frequency for smoother animation
        if self.frame_count % 100 == 0:  # Every 100 frames instead of 50
            self.fig.canvas.draw_idle()
        
        return self.lines
    
    def start_animation(self):
        """Start the live animation with performance optimizations"""
        try:
            print("Setting up animation...")
            self.anim = animation.FuncAnimation(
                self.fig, self.update_plot, interval=UPDATE_INTERVAL, 
                blit=False,  # Disable blitting to fix axis/text update issues
                cache_frame_data=False  # Don't cache frames to save memory
            )
            
            print("Starting matplotlib display...")
            plt.show(block=True)  # block=True to ensure proper event loop
            
        except KeyboardInterrupt:
            print("\nStopped by user")
        except Exception as e:
            print(f"\nError in animation: {e}")
            print("This might be due to matplotlib backend or display issues.")
            print("Try running with different backend or check your display settings.")
        finally:
            self.close()
    
    def close(self):
        """Clean shutdown"""
        print("Shutting down...")
        self.running = False
        
        try:
            if hasattr(self, 'anim') and self.anim:
                self.anim.event_source.stop()
        except Exception as e:
            print(f"Warning: Error stopping animation: {e}")
            
        try:
            if hasattr(self, 'udp_socket') and self.udp_socket:
                self.udp_socket.close()
        except Exception as e:
            print(f"Warning: Error closing UDP socket: {e}")
        
        # Properly close matplotlib
        try:
            plt.ioff()  # Turn off interactive mode
        except Exception as e:
            print(f"Warning: Error turning off interactive mode: {e}")
            
        try:
            if hasattr(self, 'fig'):
                plt.close(self.fig)
        except Exception as e:
            print(f"Warning: Error closing figure: {e}")
        
        # Print final statistics
        print("\nFinal Statistics:")
        print(f"  UDP packets received: {self.packet_count}")
        print(f"  Packet errors: {self.error_count}")
        if self.packet_count > 0:
            success_rate = ((self.packet_count - self.error_count) / self.packet_count) * 100
            print(f"  Success rate: {success_rate:.1f}%")
        print("Shutdown complete.")

def main():
    print("MCU 9D Motion Data Live Graph (UDP Receiver) - Performance Optimized")
    print("=" * 75)
    print("This will display real-time motion data in a 3x3 grid layout:")
    print("  • Column 1: Linear Velocity X, Y, Z (m/s)")
    print("  • Column 2: Angular Velocity X, Y, Z (rad/s)")  
    print("  • Column 3: Projected Gravity X, Y, Z (m/s²)")
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print(f"Performance Settings (Optimized for Smoothness):")
    print(f"  • Update rate: {1000/UPDATE_INTERVAL:.1f} FPS ({UPDATE_INTERVAL}ms interval)")
    print(f"  • Time window: {TIME_WINDOW} seconds")
    print(f"  • Buffer size: {WINDOW_SIZE} samples")
    print(f"  • Max plot points: 400 (optimized downsampling)")
    print(f"  • Auto-scaling: Every 1.5 seconds (adaptive)")
    print(f"  • Interactive backend: {matplotlib.get_backend()}")
    print("Make sure send_and_receive_mcu.py is running with ENABLE_UDP_PUBLISH = True")
    print("Close the graph window to stop.")
    print()
    
    try:
        print("Initializing velocity grapher...")
        grapher = VelocityGrapher()
        print("Starting live animation...")
        grapher.start_animation()
    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure your display is working properly")
        print("2. Try installing: pip install matplotlib")
        print("3. If using remote desktop, matplotlib might have display issues")
        print("4. Check if send_and_receive_mcu.py is running first")

if __name__ == "__main__":
    main() 