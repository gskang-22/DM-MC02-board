import serial
import struct
import time

# --- Serial Port Settings ---
# IMPORTANT: Change 'COM6' to the correct COM port for your USB-to-UART adapter
#            Check your PC's Device Manager.
SERIAL_PORT = 'COM6' # Ensure this matches your MCU's connected COM port
BAUD_RATE = 921600   # Ensure this matches your MCU's UART10 baud rate

# --- Data Settings ---
# PC sends 7 floats to MCU
PC_TO_MCU_FLOATS = 7
PC_TO_MCU_DATA_SIZE = 4 * PC_TO_MCU_FLOATS
PC_TO_MCU_MSG_SIZE = PC_TO_MCU_DATA_SIZE + 2  # 7 floats + 2 bytes CRC
PC_TO_MCU_FORMAT = '<' + 'f' * PC_TO_MCU_FLOATS  # Little-endian, 7 floats

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

def send_data(port, baud):
    try:
        # Open the serial port
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        print(f"Ready to send {PC_TO_MCU_FLOATS} floats ({PC_TO_MCU_MSG_SIZE} bytes total).")
        print("Commands:")
        print("  'test' - Send test data (1.0, 2.1, 3.2, 4.3, 5.4, 6.5, 7.6)")
        print("  'manual' - Enter 7 float values manually")
        print("  'q' - Quit")

        while True:
            user_input = input("\nEnter command: ").lower().strip()
            if user_input == 'q':
                break
            elif user_input == 'test':
                # Send test data
                test_data = [float(i + 1) + i * 0.1 for i in range(PC_TO_MCU_FLOATS)]
                # Add some special values
                test_data[0] = 1.24575764878766
                test_data[1] = 999.0
                test_data[2] = 7.0
                if PC_TO_MCU_FLOATS > 3:
                    test_data[3] = 12345.67890
                
                send_float_array(ser, test_data)
                print(f"Sent test data: {test_data}")
                
            elif user_input == 'manual':
                # Manual input of 7 floats
                float_data = []
                print(f"Enter {PC_TO_MCU_FLOATS} float values:")
                for i in range(PC_TO_MCU_FLOATS):
                    while True:
                        try:
                            value = float(input(f"Float {i+1}: "))
                            float_data.append(value)
                            break
                        except ValueError:
                            print("Invalid input. Please enter a valid number.")
                
                send_float_array(ser, float_data)
                print(f"Sent {PC_TO_MCU_FLOATS} floats: {float_data}")
                
            else:
                print("Unknown command. Use 'test', 'manual', or 'q'.")

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("Please check the COM port and ensure the device is connected.")
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        # Close the serial port when done
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print("Serial port closed.")

def send_float_array(ser, float_array):
    """Send an array of 7 floats with CRC to MCU"""
    try:
        # Pack the floats into bytes (little-endian)
        data_bytes = struct.pack(PC_TO_MCU_FORMAT, *float_array)
        
        # Calculate CRC
        crc = crc16_ccitt(data_bytes)
        
        # Create message with CRC
        msg = data_bytes + crc.to_bytes(2, 'little')
        
        # Send the message
        ser.write(msg)
        print(f"Data sent: {len(msg)} bytes (CRC: {crc:04X})")
        
    except serial.SerialException as e:
        print(f"Serial write error: {e}")
    except Exception as e:
        print(f"Error sending data: {e}")

if __name__ == "__main__":
    send_data(SERIAL_PORT, BAUD_RATE) 
