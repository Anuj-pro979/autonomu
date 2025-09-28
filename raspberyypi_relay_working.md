Alright 😄 — I’ll give you a **full setup with code + connection diagram** so your Raspberry Pi can control a relay module as a switch.

---

## **1. Hardware Connections**

Here’s the wiring for a **1-channel relay module**:

| Relay Module Pin | Raspberry Pi Pin | Description                 |
| ---------------- | ---------------- | --------------------------- |
| **VCC**          | Pin 2 (5V)       | Power supply for relay coil |
| **GND**          | Pin 6 (GND)      | Ground                      |
| **IN**           | Pin 11 (GPIO 17) | GPIO control pin            |

**Pin numbering references:**

* **BCM mode** → GPIO numbers (used in code)
* **Physical pin numbering** → pins on Pi header

---

### **Connection Diagram**

```
Raspberry Pi GPIO Header (BCM Mode)
------------------------------------------------
Pin 2 (5V)  ─── VCC (Relay Module)
Pin 6 (GND)  ─── GND (Relay Module)
Pin 11 (GPIO17) ─── IN (Relay Module)
```

**Load Side (Relay as a switch):**

```
Power Source (+) ─── NO (Normally Open relay contact)
COM (relay) ─── Device (+)
Device (-) ─── Power Source (-)
```

**How it works:**

* When GPIO17 is **ON**, relay closes COM → NO → power flows to device.
* When GPIO17 is **OFF**, relay opens COM → NO → device turns off.

---

## **2. Updated Python Code (Using gpiozero)**

Install gpiozero if not already:

```bash
sudo apt update
sudo apt install python3-gpiozero
```

Create `relay_test.py`:

```python
from gpiozero import OutputDevice
from time import sleep

# GPIO pin for relay control
RELAY_PIN = 17

# Setup relay (active_low=True because most relay modules are active-low)
relay = OutputDevice(RELAY_PIN, active_high=False, initial_value=False)

try:
    while True:
        print("Relay ON")
        relay.on()     # Turns relay ON
        sleep(2)

        print("Relay OFF")
        relay.off()    # Turns relay OFF
        sleep(2)

except KeyboardInterrupt:
    print("Program stopped")
```

---

## **3. Run the Code**

```bash
sudo python3 relay_test.py
```

You should hear the **relay click** every 2 seconds and the connected device should toggle ON/OFF.

---

⚡ **Extra Tip:**
If your relay clicks but doesn’t control the device, check **NO vs NC wiring** — for most cases you should connect your device to **NO** and **COM** so it stays OFF until activated.

---

If you want, I can now make a **visual wiring diagram image** showing the Raspberry Pi, relay module, and a lamp/motor connected so it’s crystal clear for you to build.

Do you want me to?
