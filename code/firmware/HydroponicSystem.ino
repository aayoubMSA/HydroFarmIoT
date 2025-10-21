#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// ======== WiFi & MQTT ========
const char* ssid = "Test";
const char* password = "12345678";
const char* mqtt_server = "192.168.56.71";
WiFiClient espClient;
PubSubClient client(espClient);

// ======== Pins ========
#define TRIG_PIN 5
#define ECHO_PIN 18
#define PH_PIN 34
#define TDS_PIN 35
#define TEMP_PIN 4
#define RELAY_ACIDIC 26
#define RELAY_ALKALINE 27

// ======== Water Tank Constants ========
const float TANK_HEIGHT_CM = 38.0;
const float MIN_DISTANCE_CM = 25.0;
const float USABLE_HEIGHT_CM = TANK_HEIGHT_CM - MIN_DISTANCE_CM;

// ======== pH Control Settings ========
const float PH_LOW_THRESHOLD = 5.7;
const float PH_HIGH_THRESHOLD = 6.4;
const unsigned long DOSING_DURATION_MS = 5000;
const unsigned long DOSING_COOLDOWN_MS = 60000;
unsigned long lastDoseTime = 0;
bool isDosing = false;
String dosingStatus = "Stable";

// ======== Sensor Setup ========
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
hd44780_I2Cexp lcd;

// ======== Manual Overrides (MQTT Control) ========
bool overrideAcidic = false;
bool overrideAlkaline = false;

// ======== MQTT Callback for Manual Relay Control ========
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();

  if (String(topic) == "hydrofarmiot/relay/acidic") {
    overrideAcidic = (message == "ON");
    digitalWrite(RELAY_ACIDIC, overrideAcidic ? HIGH : LOW);
    if (overrideAcidic) dosingStatus = "Manual Acid";
  }

  if (String(topic) == "hydrofarmiot/relay/alkaline") {
    overrideAlkaline = (message == "ON");
    digitalWrite(RELAY_ALKALINE, overrideAlkaline ? HIGH : LOW);
    if (overrideAlkaline) dosingStatus = "Manual Alk";
  }
}

// ======== MQTT Reconnect Logic ========
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected ✅");
      client.subscribe("hydrofarmiot/relay/acidic");
      client.subscribe("hydrofarmiot/relay/alkaline");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ======== Setup ========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("✅ HydroFarmIoT Booting...");

  // Relay Pins
  pinMode(RELAY_ACIDIC, OUTPUT);
  pinMode(RELAY_ALKALINE, OUTPUT);
  digitalWrite(RELAY_ACIDIC, LOW);
  digitalWrite(RELAY_ALKALINE, LOW);

  // Ultrasonic Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Sensors
  sensors.begin();
  lcd.begin(16, 2);
  lcd.backlight();

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println(" connected ✅");

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

// ======== Main Loop ========
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // ======== Read Water Level ========
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance_cm = (duration * 0.0343) / 2;

  float percentage_full = 0;
  if (distance_cm >= MIN_DISTANCE_CM && distance_cm <= TANK_HEIGHT_CM) {
    float water_height = TANK_HEIGHT_CM - distance_cm;
    percentage_full = (water_height / USABLE_HEIGHT_CM) * 100.0;
    percentage_full = constrain(percentage_full, 0, 100);
  }

  // ======== Read pH (Adjusted) ========
  int rawADC = analogRead(PH_PIN);
  float voltagePH = rawADC * (3.3 / 4095.0);
  float pH = (-11.02 * voltagePH + 25.45) - 0.7;  // ADJUSTED BY -0.7

  // ======== Read TDS ========
  int rawTDS = analogRead(TDS_PIN);
  float voltageTDS = rawTDS * (3.3 / 4095.0);
  float tds = (133.42 * pow(voltageTDS, 3)
              - 255.86 * pow(voltageTDS, 2)
              + 857.39 * voltageTDS) * 0.5;

  // ======== Read Water Temp ========
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // ======== Auto Dosing Control ========
  unsigned long now = millis();
  if (!overrideAcidic && !overrideAlkaline && !isDosing && (now - lastDoseTime >= DOSING_COOLDOWN_MS)) {
    if (pH > PH_HIGH_THRESHOLD) {
      digitalWrite(RELAY_ACIDIC, HIGH);
      dosingStatus = "Auto Acid";
      isDosing = true;
      lastDoseTime = now;
    } else if (pH < PH_LOW_THRESHOLD) {
      digitalWrite(RELAY_ALKALINE, HIGH);
      dosingStatus = "Auto Alk";
      isDosing = true;
      lastDoseTime = now;
    } else {
      dosingStatus = "Stable";
    }
  }

  // ======== Stop Dosing After Duration ========
  if (isDosing && (now - lastDoseTime >= DOSING_DURATION_MS)) {
    digitalWrite(RELAY_ACIDIC, LOW);
    digitalWrite(RELAY_ALKALINE, LOW);
    isDosing = false;
  }

  // ======== LCD Display ========
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("pH:");
  lcd.print(pH, 2);
  lcd.print(" ");
  lcd.print(dosingStatus.substring(0, 9));

  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(tempC, 1);
  lcd.print("C W:");
  lcd.print(percentage_full, 0);
  lcd.print("%");

  // ======== MQTT Publish ========
  client.publish("hydrofarmiot/ph", String(pH, 2).c_str());
  client.publish("hydrofarmiot/tds", String(tds, 1).c_str());
  client.publish("hydrofarmiot/temp", String(tempC, 2).c_str());
  client.publish("hydrofarmiot/water-level", String(percentage_full, 1).c_str());
  client.publish("hydrofarmiot/status", dosingStatus.c_str());

  // ======== Debug Log ========
  Serial.printf("📊 pH: %.2f | TDS: %.1f ppm | Temp: %.2f °C | Water Level: %.1f%% | %s\n\n",
                pH, tds, tempC, percentage_full, dosingStatus.c_str());

  delay(5000);  // Wait 5 seconds before next cycle
}
