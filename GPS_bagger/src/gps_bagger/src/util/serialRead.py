import serial

# Open serial port
ser = serial.Serial(
    port='COM4',         # Replace with your COM port (e.g., '/dev/ttyUSB0' for Linux)
    baudrate=115200,       # Set baud rate (match with sender's baud rate)
    timeout=1            # Set timeout for reading
)

# Check if the serial port is open
if ser.is_open:
    print(f"Connected to {ser.port} at {ser.baudrate} baud rate.")
    
    try:
        while True:
            # Read data from serial (blocking with timeout)
            data = ser.readline().decode('utf-8').strip()  # Read a line and decode
            if data:
                print(f"Received: {data}")
                
    except KeyboardInterrupt:
        print("Exiting...")
        
    finally:
        # Close the serial port
        ser.close()
        print("Serial port closed.")
else:
    print("Unable to open the serial port.")
