Got it! Since you only need **Channel 1** and **Channel 2**, I’ve made the necessary adjustments to the Python script and CSV output to capture just those two channels.

Here’s the revised version of your **Python script** that only logs **Channel 1** and **Channel 2**:

### Revised Python Script (only Channel 1 and Channel 2):

```python
import serial
import struct
import csv
import time

# Setup the serial port (adjust to your port name)
SERIAL_PORT = '/dev/ttyUSB0'  # or /dev/ttyAMA0 depending on how you're connected
BAUD_RATE = 115200  # Standard baud rate for IBUS

# Open the serial connection to the IBUS receiver
ibus = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Open CSV file for writing
csv_file = open('ibus_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Timestamp', 'Channel 1', 'Channel 2'])

def read_ibus_data():
    """Reads a frame of IBUS data and extracts PWM values."""
    # IBUS frame starts with a header byte (0x20) and has 25 bytes in total
    frame = ibus.read(25)  # Read 25 bytes from the serial connection
    
    # If a valid frame is received, decode it
    if len(frame) == 25 and frame[0] == 0x20:  # Verify start byte is 0x20
        # Extract channel data (2 bytes per channel)
        channels = []
        for i in range(8):  # Loop through all 8 channels in the IBUS frame
            pwm = struct.unpack('<H', frame[2 + (i * 2): 4 + (i * 2)])[0]
            channels.append(pwm)
        
        return channels
    else:
        return None

# Main loop to read data and log it
try:
    while True:
        # Read the IBUS data (pwm values for each channel)
        channel_values = read_ibus_data()
        
        if channel_values:
            # Get current timestamp
            timestamp = time.time()
            
            # Write only Channel 1 and Channel 2 to CSV
            csv_writer.writerow([timestamp, channel_values[0], channel_values[1]])
            print(f"Logged data: {timestamp}, Channel 1: {channel_values[0]}, Channel 2: {channel_values[1]}")
        
        # Wait before reading again
        time.sleep(0.01)  # Adjust delay for your system requirements

except KeyboardInterrupt:
    print("Exiting...")
    ibus.close()
    csv_file.close()
```

### Key Changes:

1. **CSV Columns**: The CSV file now only includes the **Timestamp**, **Channel 1**, and **Channel 2**.
2. **Data Extraction**: The script only extracts the PWM values for **Channel 1** and **Channel 2** (from the IBUS frame).
3. **Logging**: When the IBUS data is captured, only the first two channels are logged to the CSV file, and the rest are ignored.

### Expected CSV Output:

The resulting CSV will look like this:

```csv
Timestamp, Channel 1, Channel 2
1620815801.123456, 1500, 1450
1620815801.223456, 1600, 1550
1620815801.323456, 1520, 1480
...
```

### How It Works:

* The Raspberry Pi reads the IBUS data stream through the serial interface.
* For each IBUS frame, it extracts the **PWM values** for **Channel 1** and **Channel 2**.
* The data, along with a **timestamp**, is logged to a CSV file.
* The logging continues until you manually stop the program with **Ctrl+C**.

This way, you’ll have a CSV log that only includes the PWM readings for the two channels you're interested in.

Does this setup meet your needs, or would you like further adjustments?
