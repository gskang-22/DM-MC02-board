import serial
import struct
import time

# --- Serial Port Settings ---
# IMPORTANT: Change 'COM3' to the correct COM port for your USB-to-UART adapter
#            Check your PC's Device Manager.
SERIAL_PORT = 'COM6'
BAUD_RATE = 921600

# --- Data Settings ---
# Size of the data type being sent (double is typically 8 bytes)
DATA_SIZE = 8
# Format string for struct.unpack (little-endian double)
# '<' for little-endian, 'd' for double
STRUCT_FORMAT = '<d'

def receive_data(port, baud, data_size, struct_format):
    try:
        # Open the serial port
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Listening on {port} at {baud} baud...")

        while True:
            # Read the specified number of bytes
            # Check if enough bytes are available before reading
            if ser.in_waiting >= data_size:
                # Read the raw bytes
                raw_data = ser.read(data_size)

                # Check if the correct number of bytes were read
                if len(raw_data) == data_size:
                    try:
                        # Unpack the bytes into a double
                        received_value = struct.unpack(struct_format, raw_data)[0]
                        print(f"Received: {received_value}")
                    except struct.error as e:
                        print(f"Error unpacking data: {e}")
                        # Optional: Clear the input buffer if unpack fails
                        ser.reset_input_buffer()
                else:
                    print(f"Warning: Read {len(raw_data)} bytes, expected {data_size}")
                    # Optional: Clear the input buffer if read fails unexpectedly
                    ser.reset_input_buffer()
            else:
                # Small delay to prevent high CPU usage while waiting
                time.sleep(0.01)

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
    receive_data(SERIAL_PORT, BAUD_RATE, DATA_SIZE, STRUCT_FORMAT) 