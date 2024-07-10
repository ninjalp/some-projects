#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi ve MQTT bilgileri
const char* ssid = "raspiai";
const char* password = "lahmacun";
const char* mqtt_server = "192.168.113.135";
const int mqtt_port = 1883;

const char* gondericiTopic = "esp32/gonderici";
const char* aliciTopic = "esp32/alici";

WiFiClient espClient;
PubSubClient client(espClient);

// MPU6050 veri değişkenleri
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

void setup() {
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // I2C başlatma
  Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
  setupMPU();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  
  // Verileri MQTT sunucusuna gönder
  sendData();
  
  delay(100);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("WiFi'ye bağlanılıyor: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi'ye bağlandı");
  Serial.println("IP Adresi: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mesaj geldi [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT sunucusuna bağlanılıyor...");
    if (client.connect("ESP32Client")) {
      Serial.println("Bağlandı");
      client.subscribe(gondericiTopic);
    } else {
      Serial.print("Bağlantı hatası, tekrar deneyecek: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void setupMPU() {
  Wire.beginTransmission(0b1101000); // MPU'nun I2C adresi
  Wire.write(0x6B); // Güç Yönetim kaydı (Sek. 4.28)
  Wire.write(0b00000000); // SLEEP kaydını 0'a ayarlama
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); // MPU'nun I2C adresi
  Wire.write(0x1B); // Jiroskop Konfigürasyon kaydına erişim (Sek. 4.4)
  Wire.write(0b00000000); // Jiroskobu +/- 250deg./s olarak ayarlama
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); // MPU'nun I2C adresi
  Wire.write(0x1C); // İvmeölçer Konfigürasyon kaydına erişim (Sek. 4.5)
  Wire.write(0b00000000); // İvmeölçeri +/- 2g olarak ayarlama
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  accelX = (Wire.read() << 8 | Wire.read());
  accelY = (Wire.read() << 8 | Wire.read());
  accelZ = (Wire.read() << 8 | Wire.read());

  // Two's complement conversion
  if (accelX >= 0x8000) accelX = -((65535 - accelX) + 1);
  if (accelY >= 0x8000) accelY = -((65535 - accelY) + 1);
  if (accelZ >= 0x8000) accelZ = -((65535 - accelZ) + 1);

  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6);
  gyroX = (Wire.read() << 8 | Wire.read());
  gyroY = (Wire.read() << 8 | Wire.read());
  gyroZ = (Wire.read() << 8 | Wire.read());

  // Two's complement conversion
  if (gyroX >= 0x8000) gyroX = -((65535 - gyroX) + 1);
  if (gyroY >= 0x8000) gyroY = -((65535 - gyroY) + 1);
  if (gyroZ >= 0x8000) gyroZ = -((65535 - gyroZ) + 1);

  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}

void sendData() {
  String message = "aX: " + String(gForceX) + " | aY: " + String(gForceY) + " | aZ: " + String(gForceZ)
                 + " | gX: " + String(rotX) + " | gY: " + String(rotY) + " | gZ: " + String(rotZ);
  Serial.print("Mesaj gönderiliyor: ");
  Serial.println(message);
  client.publish(aliciTopic, message.c_str());
}
