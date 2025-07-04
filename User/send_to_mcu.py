import serial
import struct
import time

# --- Serial Port Settings ---
# IMPORTANT: Change 'COM5' to the correct COM port for your USB-to-UART adapter
#            Check your PC's Device Manager.
SERIAL_PORT = 'COM6' # Ensure this matches your MCU's connected COM port
BAUD_RATE = 921600   # Ensure this matches your MCU's UART10 baud rate

# --- Data Settings ---
# Format string for struct.pack (little-endian double)
# '<' for little-endian, 'd' for double
STRUCT_FORMAT = '<d'
DATA_SIZE = struct.calcsize(STRUCT_FORMAT) # Should be 8 bytes for a double

def send_data(port, baud, struct_format):
    try:
        # Open the serial port
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        print(f"Ready to send {DATA_SIZE}-byte double values.")
        print("Enter 'q' to quit.")

        while True:
            user_input = input("Enter a double value to send: ")
            if user_input.lower() == 'q':
                break

            try:
                value_to_send = float(user_input)
                # Pack the float into bytes (little-endian)
                packed_data = struct.pack(struct_format, value_to_send)

                # Send the packed bytes
                ser.write(packed_data)
                print(f"Sent: {value_to_send} ({packed_data.hex()})")
            except ValueError:
                print("Invalid input. Please enter a valid number.")
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                break

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

if __name__ == "__main__":
    send_data(SERIAL_PORT, BAUD_RATE, STRUCT_FORMAT) 
