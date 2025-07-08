import serial
import struct
import time

# --- Serial Port Settings ---
# IMPORTANT: Change 'COM6' to the correct COM port for your USB-to-UART adapter
#            Check your PC's Device Manager.
SERIAL_PORT = 'COM6'
BAUD_RATE = 921600

# --- Data Settings ---
# MCU sends 25 floats to PC
MCU_TO_PC_FLOATS = 25
MCU_TO_PC_DATA_SIZE = 4 * MCU_TO_PC_FLOATS
MCU_TO_PC_MSG_SIZE = MCU_TO_PC_DATA_SIZE + 2  # 25 floats + 2 bytes CRC
MCU_TO_PC_FORMAT = '<' + 'f' * MCU_TO_PC_FLOATS  # Little-endian, 25 floats

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

def receive_data(port, baud):
    try:
        # Open the serial port
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Listening on {port} at {baud} baud...")
        print(f"Expecting {MCU_TO_PC_FLOATS} floats ({MCU_TO_PC_MSG_SIZE} bytes total)")
        print("Press Ctrl+C to stop.")
        
        rx_count = 0
        rx_errors = 0
        consecutive_errors = 0
        max_consecutive_errors = 5  # Clear buffer after 5 consecutive errors
        
        while True:
            # Check if enough bytes are available before reading
            if ser.in_waiting >= MCU_TO_PC_MSG_SIZE:
                # Read the raw bytes
                raw_data = ser.read(MCU_TO_PC_MSG_SIZE)
                
                # Check if the correct number of bytes were read
                if len(raw_data) == MCU_TO_PC_MSG_SIZE:
                    try:
                        # Split data and CRC
                        data = raw_data[:MCU_TO_PC_DATA_SIZE]
                        crc_recv = int.from_bytes(raw_data[MCU_TO_PC_DATA_SIZE:], 'little')
                        crc_calc = crc16_ccitt(data)
                        
                        if crc_recv == crc_calc:
                            # *** CRC SUCCESS ***
                            consecutive_errors = 0  # Reset consecutive error counter
                            
                            # CRC SUCCESS - unpack the floats
                            received_floats = list(struct.unpack(MCU_TO_PC_FORMAT, data))
                            rx_count += 1
                            
                            # Extract UART status information (from MCU's perspective)
                            mcu_uart_connected = received_floats[22]     # 1.0 = connected, 0.0 = disconnected
                            mcu_time_since_last_rx = received_floats[23] # Time since MCU last received from PC (ms)
                            mcu_crc_errors = received_floats[24]         # Total CRC errors on MCU side
                            
                            # Print received data with UART status
                            status_str = "CONNECTED" if mcu_uart_connected > 0.5 else "DISCONNECTED"
                            print(f"[RX {rx_count:4d}] Data: {[f'{x:.2f}' for x in received_floats[:5]]}... {received_floats[21]:.2f}")
                            print(f"         UART Status: {status_str} | MCU Last RX: {mcu_time_since_last_rx:.0f}ms ago | MCU CRC Errors: {mcu_crc_errors:.0f}")
                            
                            # Print warning if MCU reports disconnection
                            if mcu_uart_connected < 0.5:
                                print(f"         *** WARNING: MCU reports UART DISCONNECTED! MCU will be beeping! ***")
                            
                            # Print stats every 25 successful receives (reduced frequency for more info)
                            if rx_count % 25 == 0:
                                if (rx_count + rx_errors) > 0:
                                    success_rate = (rx_count / (rx_count + rx_errors)) * 100
                                    print(f"         Stats: {rx_count} OK, {rx_errors} ERR, {success_rate:.1f}% success")
                                    print(f"         ----------------------------------------")
                        else:
                            # *** CRC ERROR - Enhanced Error Recovery ***
                            rx_errors += 1
                            consecutive_errors += 1
                            
                            print(f"[RX] CRC ERROR #{rx_errors}! Received: {crc_recv:04X}, Calculated: {crc_calc:04X}")
                            print(f"     Consecutive errors: {consecutive_errors}")
                            
                            # Check if too many consecutive errors - perform buffer recovery
                            if consecutive_errors >= max_consecutive_errors:
                                print(f"     *** BUFFER RECOVERY: Clearing input buffer after {consecutive_errors} consecutive errors ***")
                                
                                # Clear the input buffer to resynchronize
                                bytes_cleared = ser.in_waiting
                                ser.reset_input_buffer()
                                
                                print(f"     Cleared {bytes_cleared} bytes from buffer")
                                consecutive_errors = 0  # Reset counter after recovery
                                
                                # Small delay to let MCU send fresh data
                                time.sleep(0.01)
                            else:
                                # Simple recovery - try to resync by reading one byte and retry
                                print(f"     Simple recovery: Reading 1 byte to resync...")
                                if ser.in_waiting > 0:
                                    ser.read(1)  # Read and discard 1 byte to shift frame
                            
                            # Print error stats every 10 errors
                            if rx_errors % 10 == 0:
                                if (rx_count + rx_errors) > 0:
                                    success_rate = (rx_count / (rx_count + rx_errors)) * 100
                                    print(f"         Error Stats: {rx_count} OK, {rx_errors} ERR, {success_rate:.1f}% success")
                        
                    except struct.error as e:
                        print(f"Error unpacking data: {e}")
                        # Clear the input buffer if unpack fails
                        ser.reset_input_buffer()
                        print("     Cleared input buffer due to unpack error")
                else:
                    print(f"Warning: Read {len(raw_data)} bytes, expected {MCU_TO_PC_MSG_SIZE}")
                    # Clear the input buffer if read fails unexpectedly
                    ser.reset_input_buffer()
                    print("     Cleared input buffer due to incomplete read")
            else:
                # Small delay to prevent high CPU usage while waiting
                time.sleep(0.001)  # 1ms delay

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("Please check the COM port and ensure the device is connected.")
    except KeyboardInterrupt:
        print(f"\nStopped by user.")
        print(f"Final stats:")
        print(f"  Messages received: {rx_count}")
        print(f"  CRC errors: {rx_errors}")
        if (rx_count + rx_errors) > 0:
            success_rate = (rx_count / (rx_count + rx_errors)) * 100
            print(f"  Success rate: {success_rate:.1f}%")
    finally:
        # Close the serial port when done
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    receive_data(SERIAL_PORT, BAUD_RATE) 