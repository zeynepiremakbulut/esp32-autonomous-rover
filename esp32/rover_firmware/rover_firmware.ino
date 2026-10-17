#include <WiFi.h>
#include <WiFiUdp.h>

// =====================================================
// WIFI AYARLARI  (kendi bilgilerinizi yazın)
// =====================================================
const char* WIFI_SSID = "write-your-own-ssid";
const char* WIFI_PASS = "write-your-own-password";

IPAddress local_IP(192, 168, 1, 3);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// =====================================================
// UDP PORTLARI
// =====================================================
const uint16_t MOTOR_PORT = 4210;
const uint16_t LIDAR_PORT = 4211;

WiFiUDP motorUDP;
WiFiUDP lidarUDP;

IPAddress computerIP;
bool computerKnown = false;

// =====================================================
// GÜVENLİK (WATCHDOG)
// =====================================================
// Bu süre boyunca geçerli motor komutu gelmezse motorlar otomatik durur.
// PC tarafı komutu saniyede ~10 kez tekrarladığı için normal sürüşte tetiklenmez.
const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdMs = 0;
bool motorAktif = false;

// =====================================================
// TB6612 SÜRÜCÜ #1
// =====================================================
#define AIN1 4
#define AIN2 5
#define PWMA 6
#define BIN1 15
#define BIN2 16
#define PWMB 17

// TB6612 SÜRÜCÜ #2
#define AIN1_2 8
#define AIN2_2 9
#define PWMA_2 10
#define BIN1_2 11
#define BIN2_2 12
#define PWMB_2 13
#define STBY 7

// =====================================================
// LIDAR UART
// =====================================================
HardwareSerial LiDAR(1);
#define LIDAR_RX 44
#define LIDAR_TX 43
#define LIDAR_BAUD 115200

uint8_t lidarPacket[22];
int lidarIndex = 0;

// LiDAR paketleri tek tek yerine gruplanıp gönderilir (WiFi yükünü azaltır).
// PC tarafı bir datagram içinde birden fazla 22 baytlık paketi zaten işliyor.
#define LIDAR_BATCH 5
uint8_t lidarBatch[22 * LIDAR_BATCH];
int lidarBatchCount = 0;
unsigned long lastLidarSendMs = 0;
const unsigned long LIDAR_FLUSH_MS = 40;

// =====================================================
// MOTOR SÜRÜŞ
// =====================================================
void solMotor(int hiz) {
  hiz = constrain(hiz, -255, 255);
  if (hiz > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, hiz);
    digitalWrite(BIN1_2, LOW);
    digitalWrite(BIN2_2, HIGH);
    analogWrite(PWMB_2, hiz);
  } else if (hiz < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, -hiz);
    digitalWrite(BIN1_2, HIGH);
    digitalWrite(BIN2_2, LOW);
    analogWrite(PWMB_2, -hiz);
  } else {
    analogWrite(PWMA, 0);
    analogWrite(PWMB_2, 0);
  }
}

void sagMotor(int hiz) {
  hiz = constrain(hiz, -255, 255);
  if (hiz > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, hiz);
    digitalWrite(AIN1_2, HIGH);
    digitalWrite(AIN2_2, LOW);
    analogWrite(PWMA_2, hiz);
  } else if (hiz < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -hiz);
    digitalWrite(AIN1_2, LOW);
    digitalWrite(AIN2_2, HIGH);
    analogWrite(PWMA_2, -hiz);
  } else {
    analogWrite(PWMB, 0);
    analogWrite(PWMA_2, 0);
  }
}

void motorSur(int sol, int sag) {
  solMotor(sol);
  sagMotor(sag);
  motorAktif = (sol != 0 || sag != 0);
}

// =====================================================
// UDP MOTOR KOMUTLARI
// =====================================================
void komutIsle(char* buffer) {
  if (strncmp(buffer, "V:", 2) == 0) {
    int sol = 0, sag = 0;
    if (sscanf(buffer + 2, "%d,%d", &sol, &sag) == 2) {
      motorSur(sol, sag);
      lastCmdMs = millis();
    }
  } else if (strcmp(buffer, "M:w") == 0) { motorSur(150, 150);  lastCmdMs = millis(); }
  else if (strcmp(buffer, "M:s") == 0)   { motorSur(-150, -150); lastCmdMs = millis(); }
  else if (strcmp(buffer, "M:a") == 0)   { motorSur(-150, 150);  lastCmdMs = millis(); }
  else if (strcmp(buffer, "M:d") == 0)   { motorSur(150, -150);  lastCmdMs = millis(); }
  else if (strcmp(buffer, "M:x") == 0)   { motorSur(0, 0);       lastCmdMs = millis(); }
}

void udpKomutOku() {
  // Birikmiş tüm paketleri oku; sonuncusu geçerli olur (eski komutlarla geç kalma olmaz)
  for (int n = 0; n < 8; n++) {
    int packetSize = motorUDP.parsePacket();
    if (packetSize <= 0) return;

    // Bilgisayar IP'sini al ve kaydet
    computerIP = motorUDP.remoteIP();
    if (!computerKnown) {
      computerKnown = true;
      Serial.print("Bilgisayar IP baglandi: ");
      Serial.println(computerIP);
    }

    char buffer[64];
    int len = motorUDP.read(buffer, sizeof(buffer) - 1);
    if (len <= 0) continue;
    buffer[len] = '\0';
    komutIsle(buffer);
  }
}

// Komut gelmezse motorları durdur
void watchdogKontrol() {
  if (motorAktif && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    motorSur(0, 0);
    Serial.println("WATCHDOG: komut gelmedi, motorlar durduruldu");
  }
}

// =====================================================
// WIFI KONTROLÜ (koparsa motorları durdur, yeniden bağlan)
// =====================================================
unsigned long lastWifiTryMs = 0;

void wifiKontrol() {
  if (WiFi.status() == WL_CONNECTED) return;

  if (motorAktif) {
    motorSur(0, 0);
    Serial.println("WiFi koptu: motorlar durduruldu");
  }
  if (millis() - lastWifiTryMs > 3000) {
    lastWifiTryMs = millis();
    Serial.println("WiFi yeniden baglaniyor...");
    WiFi.reconnect();
  }
}

// =====================================================
// LIDAR
// =====================================================
bool lidarChecksumOK(uint8_t *packet) {
  uint32_t chk32 = 0;
  for (int i = 0; i < 20; i += 2) {
    uint16_t word = (uint16_t)packet[i] | ((uint16_t)packet[i + 1] << 8);
    chk32 = (chk32 << 1) + word;
  }
  chk32 = (chk32 & 0x7FFF) + (chk32 >> 15);
  chk32 = (chk32 & 0x7FFF) + (chk32 >> 15);
  uint16_t checksum = (uint16_t)packet[20] | ((uint16_t)packet[21] << 8);
  return ((uint16_t)chk32 == checksum);
}

void lidarBatchGonder() {
  if (lidarBatchCount == 0) return;
  if (computerKnown && WiFi.status() == WL_CONNECTED) {
    lidarUDP.beginPacket(computerIP, LIDAR_PORT);
    lidarUDP.write(lidarBatch, 22 * lidarBatchCount);
    lidarUDP.endPacket();
  }
  lidarBatchCount = 0;
  lastLidarSendMs = millis();
}

void lidarPaketEkle(uint8_t *pkt) {
  memcpy(lidarBatch + 22 * lidarBatchCount, pkt, 22);
  lidarBatchCount++;
  if (lidarBatchCount >= LIDAR_BATCH) lidarBatchGonder();
}

// LiDAR okuma (döngü kilitlenmesini önlemek için limitli)
void lidarOku() {
  int readCount = 0;
  while (LiDAR.available() > 0 && readCount < 128) {
    uint8_t b = LiDAR.read();
    readCount++;

    if (lidarIndex == 0) {
      if (b == 0xFA) {
        lidarPacket[0] = b;
        lidarIndex = 1;
      }
      continue;
    }

    if (lidarIndex == 1) {
      if (b >= 0xA0 && b <= 0xF9) {
        lidarPacket[1] = b;
        lidarIndex = 2;
      } else {
        lidarIndex = (b == 0xFA) ? 1 : 0;
        if (lidarIndex == 1) lidarPacket[0] = 0xFA;
      }
      continue;
    }

    lidarPacket[lidarIndex++] = b;

    if (lidarIndex >= 22) {
      if (lidarChecksumOK(lidarPacket)) {
        lidarPaketEkle(lidarPacket);
      }
      lidarIndex = 0;
    }
  }

  // Veri yavaşlarsa bekleyen paketleri geciktirmeden gönder
  if (lidarBatchCount > 0 && millis() - lastLidarSendMs > LIDAR_FLUSH_MS) {
    lidarBatchGonder();
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);

  // Motor pinleri
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1_2, OUTPUT); pinMode(AIN2_2, OUTPUT); pinMode(PWMA_2, OUTPUT);
  pinMode(BIN1_2, OUTPUT); pinMode(BIN2_2, OUTPUT); pinMode(PWMB_2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Sürücüyü aktif tut
  motorSur(0, 0);

  // LiDAR UART (büyük RX tamponu: WiFi gönderimi sırasında bayt kaybını önler)
  LiDAR.setRxBufferSize(2048);
  LiDAR.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }
  WiFi.setSleep(false);   // WiFi güç tasarrufu gecikme ve paket kaybı yaratır

  Serial.println("WiFi Baglandi!");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  motorUDP.begin(MOTOR_PORT);
  lidarUDP.begin(LIDAR_PORT);
  lastCmdMs = millis();
}

void loop() {
  wifiKontrol();
  udpKomutOku();
  watchdogKontrol();
  lidarOku();
}
