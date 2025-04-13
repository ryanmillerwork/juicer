#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <driver/ledc.h>


// speedup recs:
// set baud to 2000000 x
// use Serial.flush(); after each write x
// Serial.setTxBufferSize(128); x
// Serial.setRxBufferSize(128);  x
// connect to usb 3.0
// sudo setserial /dev/ttyACM0 low_latency


// Define constants
const int PULSES_PER_STEP = 32;
const int STEPS_PER_REV = 200;
const int MAX_RPS = 8;
const int MAX_PWM_FREQ_MOTOR = PULSES_PER_STEP * STEPS_PER_REV * MAX_RPS;

float flow_rate;                  // This is the empirically determined flow rate given the target_rps, stored in flash
float purge_vol;                  // Volume to purge when pressing the purge button, stored in flash
int reward_number = 0;            // Number of rewards since being reset
float reward_mls = 0.0;           // Volume of rewards since being reset 
bool purging = false;             // Goes true while purging
bool manual_watering = false;     // Goes true while pressing water button
bool remote_watering = false;     // Goes true while digital line is high
bool serial_watering = false;     // Goes true while watering requested by serial port
bool calibration_in_progress = false;
int calibration_n = 0;
int calibration_on = 0;
int calibration_off = 0;
int calibration_count = 0;
unsigned long calibration_start_time = 0;
float serial_vol = 0.0;           // Keep track of this in case it needs to be subtracted for overlapping rewards
float target_rps;                 // Target rotations per second of the pump head. max is around 3-4
int target_hz;                    // Frequency of pulses to achieve desired target_rps
bool reset_pressed = false;       // Keeps track of button press to only execute on button down
unsigned long water_start_time = 0; // Keeps track of when you first press manual start or remote start so we can track amount given
bool pump_running = false;        // Keeps track of if pump is running
uint32_t pump_stop_time = 0;      // What time to stop the pump
bool sched_disp_update = false;   // For when you want to update the display but not quite yet

const int pwmChannel = 0;
const int pwmResolution = 8; // 8-bit resolution
// int stepChannel = -1; // global variable to hold the LEDC channel for the STEP_PIN

const int LEDC_CHANNEL = 0;  // LEDC channel (0-7 for ESP32-S2)
const int LEDC_TIMER_BIT = 8; // 8-bit timer resolution
const int LEDC_BASE_FREQ = 100000; // Base frequency in Hz
const int LEDC_RESOLUTION = 8; // 8-bit resolution
const int stepChannel = 0;  // Use a valid LEDC channel (0-7 for ESP32-S2)
const uint8_t ledcChannel = 0;  // Using channel 0




// Display, NeoPixel, flash setup
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Preferences preferences;

/*
 * Available API Commands:
 *
 * 1. Set Commands:
 *    - {"set": {"flow_rate": <float>}}
 *      - Sets the flow rate (must be > 0).
 *    - {"set": {"purge_vol": <float>}}
 *      - Sets the purge volume (must be > 0).
 *    - {"set": {"target_rps": <float>}} {"set": {"target_rps": 3}}
 *      - Sets the target revolutions per second (must be > 0 and <= MAX_RPS).
 *    - {"set": {"voltage_mult": <float>}} {"set": {"voltage_mult": 11.909}}
 *      - Sets the multiplier to account for the voltage divider ratio
 *
 * 2. Do Commands (only 1 allowed per request):
 *    - {"do": "abort"}
 *      - Aborts the current operation, stops the pump, and updates reward metrics.
 *    - {"do": {"reward": <float>}} {"do": {"reward": 0.8}}
 *      - Dispenses a reward of the specified amount (must be > 0).
 *    - {"do": {"purge": <float>}}
 *      - Starts a purge operation for the specified amount (must be > 0).
 *    - {"do": {"calibration": {"n": <int>, "on": <int>, "off": <int>}}} {"do": {"calibration": {"n": 10, "on": 500, "off": 500}}}
 *      - Runs a calibration cycle with:
 *        - n: Number of cycles (must be > 0).
 *        - on: Duration in milliseconds for the pump to be ON (must be > 0).
 *        - off: Duration in milliseconds for the pump to be OFF (must be > 0).
 *
 * 3. Get Commands:
 *    - {"get": ["flow_rate"]}
 *      - Retrieves the current flow rate.
 *    - {"get": ["purge_vol"]}
 *      - Retrieves the current purge volume.
 *    - {"get": ["target_rps"]}
 *      - Retrieves the current target revolutions per second.
 *    - {"get": ["reward_mls"]}
 *      - Retrieves the total amount of reward dispensed in milliliters.
 *    - {"get": ["reward_number"]}
 *      - Retrieves the total number of rewards dispensed.
 *    - {"get": ["pump_voltage"]}
 *      - Retrieves the current voltage supplied to the pump
 *    - {"get": ["voltage_mult"]}
 *      - Retrieves the multiplier to account for the voltage divider ratio
 *    - {"get": ["<unknown_parameter>"]}
 *      - Returns "Unknown parameter" for any unrecognized parameter.
 *
 * Example Combined Request:
 *    {"set": {"target_rps": 2, "flow_rate": 0.65}, "do": {"reward": 1},"get": ["reward_mls", "reward_number"]}
 *
 * Expected Response:
 *    {
 *      "status": "success",
 *      "reward_mls": <float>,
 *      "reward_number": <int>
 *    }
 */


// Pin definitions for motor and switches

// old
// #define REMOTE_TOGGLE_PIN 13
// #define DMODE0_PIN 12
// #define DMODE1_PIN 11
// #define DMODE2_PIN 10
// #define DIR_PIN 9
// #define STEP_PIN 6
// #define EN_PIN 5

// new
#define REMOTE_TOGGLE_PIN 13

#define DMODE0_PIN 5
#define DMODE1_PIN 6
#define DMODE2_PIN 9
#define DIR_PIN 10
#define STEP_PIN 11
#define EN_PIN 12

#define SWITCH_D0_PIN 0  // D0 is also used as the BOOT button, pulled HIGH by default
#define SWITCH_D1_PIN 1
#define SWITCH_D2_PIN 2

// Values for monitoring the pump voltage
const int voltageReadingPin = A1;   // Analog input pin from the voltage divider.
const int NUM_READINGS = 25;        // Number of readings to take the median of
float voltageSamples[NUM_READINGS]; // Array for storing the readings
int voltageSampleIndex = 0;         // Keeps track of which reading we're on
const float ADC_TO_VOLT = 0.0008059;// 3.3 / 4095.0;  Conversion factor: ADC reading to voltage.
float voltage_mult = 11.4;     // Multiplier to recover the full-scale voltage.  11.908
unsigned long lastVoltageReadingTime = 0; // Keeps track of last reading so we dont do it too often


void writeTextToScreen(int x, int y, uint16_t color, String text) {
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.println(text);
}

void update_display(bool highlight_flow = false, bool highlight_purge = false) {
  tft.fillScreen(ST77XX_BLACK);

  if (purging) tft.fillRect(0, 29, 240, 24, 0x0020a8);  // Background rectangle
  if (manual_watering || remote_watering || serial_watering) tft.fillRect(0, 5, 240, 24, 0x0020a8); // Background rectangle

  writeTextToScreen(10, 20, ST77XX_WHITE, "Flow rate: " + String(flow_rate) + " mL/s");
  writeTextToScreen(10, 44, ST77XX_WHITE, "Purge Vol: " + String(purge_vol) + " mL");
  tft.drawLine(10, 68, 230, 68, ST77XX_WHITE);
  writeTextToScreen(10, 92, ST77XX_WHITE, "Reward #: " + String(reward_number));
  writeTextToScreen(10, 116, ST77XX_WHITE, "Reward mLs: " + String(reward_mls, 3));
}

void start_pump(uint32_t color = pixels.Color(255, 255, 255)) {
  digitalWrite(DMODE0_PIN, HIGH), digitalWrite(DMODE1_PIN, HIGH), digitalWrite(DMODE2_PIN, HIGH);
  // ledcWrite(stepChannel, 128);  // Duty cycle for motor speed
  ledcWrite(STEP_PIN, 128);
  // ledcWrite(stepChannel, 128);  // To start the pump

  pixels.fill(color);
  pixels.show();
  pump_running = true;
}

void stop_pump() {
  ledcWrite(STEP_PIN, 0);
  // ledcWrite(stepChannel, 0);  // To start the pump

  digitalWrite(DMODE0_PIN, LOW), digitalWrite(DMODE1_PIN, LOW), digitalWrite(DMODE2_PIN, LOW);
  pixels.clear();
  pixels.show();
  pump_running = false;
}

void handle_reward(float reward_value, uint32_t color) {
  if (serial_watering) {
    reward_mls -= serial_vol; 
    reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
  }
  serial_vol = reward_value;
  serial_watering = true;
  water_start_time = millis();
  start_pump(color);
  pump_stop_time = millis() + reward_value / flow_rate * 1000;
  reward_mls += reward_value;
  reward_number++;
  sched_disp_update = true;
}

void handle_calibration(int n, int on, int off) {
  calibration_in_progress = true;
  calibration_n = n;
  calibration_on = on;
  calibration_off = off;
  calibration_count = 0;
  calibration_start_time = millis();
  start_pump(pixels.Color(0, 255, 0));
  reward_mls += calibration_on / 1000.0 * flow_rate;
  reward_number++;
  water_start_time = millis();
  update_display();
}

void check_buttons() {
  // Check for button down for reset
  if (digitalRead(SWITCH_D0_PIN) == LOW && !reset_pressed) {
    reset_pressed = true;
    reward_number = 0;
    reward_mls = 0;
    update_display();
  } else if (digitalRead(SWITCH_D0_PIN) == HIGH) {
    reset_pressed = false;  // Reset state when button is released
  }

  // Purge button (S1)
  if (digitalRead(SWITCH_D1_PIN) == HIGH && !purging && !remote_watering && !manual_watering) {
    purging = true;
    pump_stop_time = millis() + purge_vol / flow_rate * 1000;
    start_pump(pixels.Color(255, 255, 0));
    update_display(false, true); // Highlight purge line
  }

  // Manual water button (S2)
  if (digitalRead(SWITCH_D2_PIN) == HIGH && !purging && !remote_watering && !manual_watering) {
    manual_watering = true;
    water_start_time = millis();
    start_pump(pixels.Color(255, 0, 255));
    update_display(true, false); // Highlight flow rate line
  }

  // Remote water toggle
  if (digitalRead(REMOTE_TOGGLE_PIN) == HIGH && !purging && !remote_watering && !manual_watering) {
    remote_watering = true;
    water_start_time = millis();
    start_pump(pixels.Color(0, 255, 255));
    update_display(true, false); // Highlight flow rate line
  }
}

void check_for_pump_stop() {
  if (purging && (millis() >= pump_stop_time)) {
    purging = false;
    stop_pump();
    update_display(); // Reset highlight
  }

  if (serial_watering && (millis() >= pump_stop_time)) {
    // reward_mls += serial_vol;  // Ensure reward_mls is updated correctly
    // reward_number++;
    serial_watering = false;
    stop_pump();
    update_display(); // Reset highlight
  }

  if (manual_watering && !digitalRead(SWITCH_D2_PIN)) {
    // Button released, stop manual watering and calculate reward_mls
    reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
    manual_watering = false;
    stop_pump();
    update_display(); // Reset highlight
  }

  if (remote_watering && !digitalRead(REMOTE_TOGGLE_PIN)) {
    // Line went to zero, stop pump and calculate reward_mls
    reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
    remote_watering = false;
    reward_number++;
    stop_pump();
    update_display(); // Reset highlight
  }

  if (calibration_in_progress) {
    if (calibration_count < calibration_n) {
      if (pump_running && millis() >= calibration_start_time + calibration_on) {
        stop_pump();
        calibration_start_time = millis();
        update_display(); // Reset highlight
      } else if (!pump_running && millis() >= calibration_start_time + calibration_off) {
        calibration_count++;
        if (calibration_count < calibration_n) {
          start_pump(pixels.Color(0, 255, 0));
          water_start_time = millis();
          reward_mls += calibration_on / 1000.0 * flow_rate;
          reward_number++;
          calibration_start_time = millis();
          update_display(); // Reset highlight
        } else {
          calibration_in_progress = false;
          update_display(); // Reset highlight
        }
      }
    }
  }
}

void check_serial_commands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    StaticJsonDocument<200> doc;
    StaticJsonDocument<400> responseDoc; // Response JSON object to collect status
    DeserializationError error = deserializeJson(doc, command);
    
    if (error) {
      Serial.println("{\"status\": \"Invalid JSON format\"}");
      return;
    }

    // Validate that there is at most one "do" operation
    if (doc.containsKey("do")) {
      if (doc["do"].is<JsonObject>()) {
        JsonObject doParams = doc["do"].as<JsonObject>();
        int doCount = 0;
        for (JsonPair keyValue : doParams) {
          doCount++;
        }
        if (doCount > 1) {
          responseDoc["status"] = "failure";
          responseDoc["error"] = "Only one 'do' operation is allowed in the request";
          String response;
          serializeJson(responseDoc, response);
          Serial.println(response);
          return;
        }
      } else if (!doc["do"].is<const char*>()) {
        responseDoc["status"] = "failure";
        responseDoc["error"] = "Invalid 'do' command format";
        String response;
        serializeJson(responseDoc, response);
        Serial.println(response);
        return;
      }
    }

    // Handle "set" operations
    if (doc.containsKey("set")) {
      bool success = true;
      JsonObject setParams = doc["set"].as<JsonObject>();

      if (setParams.containsKey("flow_rate")) {
        float new_flow_rate = setParams["flow_rate"].as<float>();
        if (new_flow_rate > 0) {
          flow_rate = new_flow_rate;
          preferences.putFloat("flow_rate", flow_rate);
          sched_disp_update = true;
        } else {
          success = false;
          responseDoc["error"] = "Invalid flow_rate value";
        }
      }
      
      if (setParams.containsKey("voltage_mult")) {
        float new_voltage_mult = setParams["voltage_mult"].as<float>();
        if (new_voltage_mult > 0) {
          voltage_mult = new_voltage_mult;
          preferences.putFloat("voltage_mult", voltage_mult);
          preferences.end();  // Force commit changes
          preferences.begin("watering", false);  // Reopen the namespace
        } else {
          success = false;
          responseDoc["error"] = "Invalid voltage_mult value";
        }
      }

      if (setParams.containsKey("purge_vol")) {
        float new_purge_vol = setParams["purge_vol"].as<float>();
        if (new_purge_vol > 0) {
          purge_vol = new_purge_vol;
          preferences.putFloat("purge_vol", purge_vol);
          sched_disp_update = true;
        } else {
          success = false;
          responseDoc["error"] = "Invalid purge_vol value";
        }
      }

      if (setParams.containsKey("target_rps")) {
        float new_target_rps = setParams["target_rps"].as<float>();
        if (new_target_rps > 0 && new_target_rps <= MAX_RPS) {
          target_rps = new_target_rps;
          target_hz = min(static_cast<int>(target_rps * PULSES_PER_STEP * STEPS_PER_REV), MAX_PWM_FREQ_MOTOR);
          
          ledcDetach(STEP_PIN);  // Detach the current configuration
          ledcAttachChannel(STEP_PIN, target_hz, pwmResolution, ledcChannel);


          
        } else {
          success = false;
          responseDoc["error"] = "target_rps out of range";
        }
        sched_disp_update = true;
      }

      responseDoc["status"] = success ? "success" : "failure";
    }

    // Handle "do" operations
    if (doc.containsKey("do")) {
      bool success = true;

      // Check if "do" is a simple string command (e.g., {"do": "abort"})
      if (doc["do"].is<const char*>()) {
        String action = doc["do"].as<String>();
        if (action == "abort") {
          stop_pump();
          if (serial_watering || calibration_in_progress) {
            if (serial_watering) reward_mls -= serial_vol; 
            if (calibration_in_progress) reward_mls -= calibration_on / 1000.0 * flow_rate;
            reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
            serial_watering = false;
            calibration_in_progress = false;
          }
          sched_disp_update = true;
        } else {
          success = false;
          responseDoc["error"] = "Unknown action";
        }
      }
      // Handle "do" with parameters (e.g., {"do": {"reward": 5}})
      else if (doc["do"].is<JsonObject>()) {
        JsonObject doParams = doc["do"].as<JsonObject>();
        bool validCommand = false;

        if (doParams.containsKey("reward")) {
          validCommand = true;
          float reward_value = doParams["reward"].as<float>();
          if (reward_value > 0) {
            handle_reward(reward_value, pixels.Color(255, 255, 255));
          } else {
            success = false;
            responseDoc["error"] = "Invalid reward value";
          }
        }

        if (doParams.containsKey("purge")) {
          validCommand = true;
          float purge_amount = doParams["purge"].as<float>();
          if (purge_amount > 0) {
            purging = true;
            pump_stop_time = millis() + purge_amount / flow_rate * 1000;
            start_pump(pixels.Color(255, 255, 0));
            sched_disp_update = true;
          } else {
            success = false;
            responseDoc["error"] = "Invalid purge amount";
          }
        }

        if (doParams.containsKey("calibration")) {
          validCommand = true;
          JsonObject calibrationParams = doParams["calibration"].as<JsonObject>();
          int n = calibrationParams["n"].as<int>();
          int on = calibrationParams["on"].as<int>();
          int off = calibrationParams["off"].as<int>();

          if (n > 0 && on > 0 && off > 0) {
            handle_calibration(n, on, off);
          } else {
            success = false;
            responseDoc["error"] = "Invalid calibration parameters.";
          }
        }

        if (!validCommand) {
          success = false;
          responseDoc["error"] = "Invalid 'do' command format. Use 'abort' as a string, or valid parameters.";
        }
      }

      responseDoc["status"] = success ? "success" : "failure";
    }

    // Handle "get" operations
    if (doc.containsKey("get")) {
      JsonArray getArray = doc["get"].as<JsonArray>();

      for (JsonVariant value : getArray) {
        String param = value.as<String>();

        if (param == "flow_rate") responseDoc["flow_rate"] = flow_rate;
        else if (param == "purge_vol") responseDoc["purge_vol"] = purge_vol;
        else if (param == "target_rps") responseDoc["target_rps"] = target_rps;
        else if (param == "reward_mls") responseDoc["reward_mls"] = reward_mls;
        else if (param == "reward_number") responseDoc["reward_number"] = reward_number;
        else if (param == "voltage_mult") responseDoc["voltage_mult"] = voltage_mult;
        else if (param == "pump_voltage") responseDoc["pump_voltage"] = compute_voltage_median();
        else responseDoc[param] = "Unknown parameter";
      }
    }

    // Send the response JSON
    String response;
    serializeJson(responseDoc, response);
    Serial.println(response);
    Serial.flush(); // intended to speed up the response

    if (sched_disp_update) {
      update_display();
      sched_disp_update = false;
    }
  }
}

void take_voltage_reading() {
  if ((millis() - lastVoltageReadingTime) > 20) {
    voltageSamples[voltageSampleIndex] = analogRead(voltageReadingPin) * ADC_TO_VOLT * voltage_mult;
    voltageSampleIndex++;
    if (voltageSampleIndex >= NUM_READINGS) {voltageSampleIndex = 0;}
    lastVoltageReadingTime = millis();
  }
}

float compute_voltage_median() {
  float tempArr[NUM_READINGS];
  // Copy the array so we do not modify the original values.
  for (int i = 0; i < NUM_READINGS; i++) {
    tempArr[i] = voltageSamples[i];
  }
  
  // Simple bubble sort.
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = 0; j < NUM_READINGS - i - 1; j++) {
      if (tempArr[j] > tempArr[j + 1]) {
        float temp = tempArr[j];
        tempArr[j] = tempArr[j + 1];
        tempArr[j + 1] = temp;
      }
    }
  }
  
  // For an odd-sized array, the median is the middle element.
  return tempArr[NUM_READINGS / 2];  // For n==25, this returns element index 12.
}




void setup() {

  Serial.begin(2000000);
  while(!Serial) delay(10);
  Serial.setRxBufferSize(128);  // intended to speed things up. make sure its not too small...  Set RX buffer to 128 bytes for 128 characters max, assuming UTF-8

  // Setup preferences
  preferences.begin("watering", false); // open the watering namespace in read-write mode
  flow_rate = preferences.getFloat("flow_rate", 0.5);
  purge_vol = preferences.getFloat("purge_vol", 5.0);
  target_rps = preferences.getFloat("target_rps", 3.0);
  voltage_mult = preferences.getFloat("voltage_mult", 11.909);
  target_hz = min(static_cast<int>(target_rps * PULSES_PER_STEP * STEPS_PER_REV), MAX_PWM_FREQ_MOTOR);

  // Print debug info after configuration
  Serial.print("Initial target_rps: ");
  Serial.println(target_rps);
  Serial.print("Initial target_hz: ");
  Serial.println(target_hz);
  Serial.print("Initial voltage_mult: ");
  Serial.println(voltage_mult);

  // Setup backlight and power
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // Initialize display
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&FreeMono9pt7b);
  tft.setTextWrap(false);
  update_display(false, false);

  // Set up motor control pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DMODE0_PIN, OUTPUT);
  pinMode(DMODE1_PIN, OUTPUT);
  pinMode(DMODE2_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(REMOTE_TOGGLE_PIN, INPUT_PULLDOWN);
  pinMode(SWITCH_D0_PIN, INPUT_PULLUP); // D0 is pulled HIGH by default
  pinMode(SWITCH_D1_PIN, INPUT_PULLDOWN);
  pinMode(SWITCH_D2_PIN, INPUT_PULLDOWN);

  digitalWrite(EN_PIN, HIGH);
  digitalWrite(DMODE0_PIN, LOW);
  digitalWrite(DMODE1_PIN, LOW);
  digitalWrite(DMODE2_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);

  // Set the pin mode and ensure the pin is LOW initially
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  // Attach the pin to LEDC using an explicit channel
  ledcAttachChannel(STEP_PIN, target_hz, pwmResolution, ledcChannel);
  delay(10);

  // Analog pin for reading pump supply voltage
  // analogSetWidth(12);
  // analogSetPinAttenuation(voltageReadingPin, ADC_ATTENDB_11);
  analogSetPinAttenuation(voltageReadingPin, ADC_11db);

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(75);
  pixels.show();
}

void loop() {
  check_buttons();
  check_serial_commands();
  check_for_pump_stop();
  take_voltage_reading();
}
