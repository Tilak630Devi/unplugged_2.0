
## Hardware Setup

### Battery & Charging Section
- **Solar Panel:** Supplies ~5–6 V (via a blocking diode) to a common charging node.
- **TP4056 Modules (×2):** Charge Battery A and Battery B separately.
- **Li‑ion Batteries (×2):** 3.7 V nominal, 4.2 V fully charged.
- **Battery Selector (Relay):**  
  - **NC:** Connected to Battery A’s positive terminal.  
  - **NO:** Connected to Battery B’s positive terminal.  
  - **COM:** Feeds the active battery output to the MAX471 sensor.
- **MAX471 Sensor:** Measures current/voltage drop from the active battery.  
  - **Rs+:** Connected to the battery selector’s COM (active battery).  
  - **Rs–:** Connected to the boost converter’s input.  
  - **Analog Output (Vo):** Connected to an ESP32 ADC pin (e.g., GPIO10) for monitoring.
- **Boost Converter:** Steps up the voltage to 5 V.
- **Hold‑Up Capacitor:** A large low‑ESR capacitor (e.g., 1000–2200 µF, ≥10 V) across the boost converter output and GND to stabilize the 5 V rail during battery switching.

### Sensor Modules & Communication
- **Analog Sensors (MQ‑135, Sound Sensor, Dust Sensor, Soil Moisture, Hall Effect):** Connected to designated ADC pins.
- **I2C Sensors (OLED Display, BME280, RTC DS3231):**  
  - **SDA:** GPIO8  
  - **SCL:** GPIO9
- **Digital Sensors (IR Flame Sensor, Relay for actuation, DHT22, etc.):** Connected to assigned digital GPIOs.
- **SPI Sensor (RC522 RFID):**  
  - **SCLK:** GPIO19  
  - **MISO:** GPIO20  
  - **MOSI:** GPIO21  
  - **CS:** GPIO45  
  - **RST:** GPIO18 (optional, as per wiring)
- **UART Sensor (GPS – NEO‑7M):**  
  - **RX:** GPIO43  
  - **TX:** GPIO44

### Relay Module Detailed Connections
| Relay Module Pin         | Connection                                                                     | Notes                                                     |
|--------------------------|--------------------------------------------------------------------------------|-----------------------------------------------------------|
| **Vcc**                  | 5 V rail (from boost converter output)                                         | Must match relay coil voltage (typically 5 V)             |
| **GND**                  | Common ground (shared by TP4056 modules, batteries, ESP32, etc.)                |                                                           |
| **IN (Control Signal)**  | ESP32 GPIO (e.g., GPIO22)                                                      | LOW = Battery A (NC active), HIGH = Battery B (NO active)   |
| **COM (Common Contact)** | Connected to the boost converter’s input (via MAX471 sensor output path)         | This terminal feeds the active power to the load.         |
| **NC (Normally Closed)** | Connected to Battery A’s positive terminal (TP4056_A output)                    | Default state when relay is not energized                 |
| **NO (Normally Open)**   | Connected to Battery B’s positive terminal (TP4056_B output)                    | Engaged when relay is activated to switch power source     |

## Bill of Materials (BOM)
Refer to the [BOM section](#bill-of-materials) in the project documentation for a detailed parts list with specifications and purchase links. (Key items include the ESP32‑S3 DevKitC board, sensor modules, TP4056 charger modules, Li‑ion batteries, MAX471 sensor, boost converter, relay module, and supporting passive components.)

## Installation & Setup

1. **Hardware Assembly:**
   - Assemble the PCB according to the KiCad design files.
   - Solder all through-hole components.
   - Wire sensors, battery circuits, and the relay module as detailed in the Hardware Setup section.
   - Ensure all grounds are common and verify proper voltage levels with a multimeter before powering the ESP32.

2. **Software Setup:**
   - Install the Arduino IDE.
   - Install necessary libraries via the Library Manager (e.g., Adafruit_GFX, Adafruit_SSD1306, Adafruit_BME280, RTClib, DHT, MFRC522, TinyGPS++ etc.).
   - Load the provided Arduino sketches (see Code Examples section) for sensor interfacing and battery management.

3. **Programming & Calibration:**
   - Upload the code to the ESP32‑S3 DevKitC.
   - Calibrate sensor thresholds (battery voltage, current readings from MAX471) based on your actual setup.
   - Test individual sensor outputs and verify that the relay switches correctly between batteries when required.

## Code Examples
The repository includes Arduino sketches for:
- **MQ‑135, Sound, Dust, Soil Moisture, Hall Effect, Rain Sensors**
- **OLED Display (I2C)**
- **BME280 Environmental Sensor**
- **RTC DS3231**
- **IR Flame Sensor**
- **Water Flow Sensor**
- **DHT22 Sensor**
- **RC522 RFID**
- **GPS (NEO‑7M)**
- **Battery Management & Relay Control**

Refer to the `/code` folder for detailed code files. An example for battery management and relay control is shown below:

```cpp
// Battery Management & Relay Control Example
const int max471Pin = 10;         // ADC pin for MAX471 analog output
const int relayControlPin = 22;   // Relay control pin for battery selector
const int batteryVoltagePin = A0; // ADC pin for battery voltage reading (via voltage divider)

const float voltageThreshold = 3.6;  // Example threshold voltage (adjust based on calibration)

void setup() {
  Serial.begin(115200);
  pinMode(relayControlPin, OUTPUT);
  digitalWrite(relayControlPin, LOW); // Default: Battery A active (NC connected)
}

void loop() {
  // Read battery voltage (assuming voltage divider halves the voltage)
  int battRaw = analogRead(batteryVoltagePin);
  float batteryVoltage = battRaw * (3.3 / 4095.0) * 2;
  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);

  // Read load sensing from MAX471
  int max471Raw = analogRead(max471Pin);
  float max471Voltage = max471Raw * (3.3 / 4095.0);
  Serial.print("MAX471 Voltage: ");
  Serial.println(max471Voltage);

  // Decision logic: switch battery if voltage is below threshold
  if (batteryVoltage < voltageThreshold) {
    digitalWrite(relayControlPin, HIGH); // Switch to Battery B (NO active)
    Serial.println("Switching to Battery B...");
    delay(3000); // Allow time for relay switching and capacitor hold-up
  } else {
    digitalWrite(relayControlPin, LOW);  // Use Battery A (default NC)
  }
  
  delay(2000);
}
