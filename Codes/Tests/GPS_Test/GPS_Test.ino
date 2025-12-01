#include <Wire.h>
#include <TinyGPSPlus.h>

#define ENDERECO_BUSSOLA 0x1E
#define REG_CONFIG_A 0x00
#define REG_CONFIG_B 0x01
#define REG_MODO     0x02
#define REG_DADOS    0x03

TinyGPSPlus gps;
unsigned long lastSend = 0; // control de 500 ms

// --- Funciones brújula ---
bool inicializar_bussola() {
  Wire.beginTransmission(ENDERECO_BUSSOLA);
  Wire.write(REG_CONFIG_A);
  Wire.write(0x70);
  if (Wire.endTransmission() != 0) { Serial.println(">>> ERRO: Config A bússola!"); return false; }

  Wire.beginTransmission(ENDERECO_BUSSOLA);
  Wire.write(REG_CONFIG_B);
  Wire.write(0xA0);
  if (Wire.endTransmission() != 0) { Serial.println(">>> ERRO: Config B bússola!"); return false; }

  Wire.beginTransmission(ENDERECO_BUSSOLA);
  Wire.write(REG_MODO);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) { Serial.println(">>> ERRO: Modo bússola!"); return false; }

  Serial.println(">>> Bússola HMC5883L inicializada");
  return true;
}

bool ler_dados_bussola(float* heading) {
  int16_t x, y, z;
  Wire.beginTransmission(ENDERECO_BUSSOLA);
  Wire.write(REG_DADOS);
  if (Wire.endTransmission() != 0) return false;

  Wire.requestFrom(ENDERECO_BUSSOLA, 6);
  if (Wire.available() != 6) return false;

  x = (Wire.read() << 8) | Wire.read();
  z = (Wire.read() << 8) | Wire.read();
  y = (Wire.read() << 8) | Wire.read();

  float angulo = atan2((float)y, (float)x);
  if (angulo < 0) angulo += 2 * PI;
  *heading = (angulo * 180.0 / PI) + 180;
  return true;
}

// --- Calibración brújula (pares leitura->real) ---
const int NUM_PONTOS = 9;  // tienes 9 valores en la tabla
double tabela_leitura[NUM_PONTOS] = {301.4, 280.4, 260.6, 240.3, 219.2, 197.1, 174.3, 148.4, 114.1};
double tabela_real[NUM_PONTOS]    = {60,    90,    120,   150,   180,   210,   240,   270,   300};

double corrigir_heading(double leitura) {
  // Normalizar al rango [0,360)
  while (leitura < 0) leitura += 360;
  while (leitura >= 360) leitura -= 360;

  // Buscar el intervalo
  for (int i = 0; i < NUM_PONTOS - 1; i++) {
    if (leitura >= tabela_leitura[i] && leitura <= tabela_leitura[i+1]) {
      double frac = (leitura - tabela_leitura[i]) / (tabela_leitura[i+1] - tabela_leitura[i]);
      return tabela_real[i] + frac * (tabela_real[i+1] - tabela_real[i]);
    }
  }

  // Si no cae en un rango (p.ej. cerca de 0º/360º), devolver leitura sin tocar
  return leitura;
}

// --- Setup ---
void setup() {
  Serial.begin(9600);   // Debug
  Serial2.begin(9600);  // GPS en RX2
  Serial3.begin(9600);  // TX3 -> Nano

  Wire.begin();

  if (!inicializar_bussola()) {
    Serial.println("❌ Error inicializando brújula");
  }

  Serial.println("Mega listo, esperando fix GPS...");
}

// --- Loop ---
void loop() {
  // --- Leer GPS siempre ---
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }

  // --- Enviar datos cada 500 ms ---
  if (millis() - lastSend >= 500) {
    lastSend = millis();

    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

      // --- Leer brújula ---
      float headingDeg = 0;
      if (!ler_dados_bussola(&headingDeg)) {
        Serial.println("❌ Error leyendo brújula");
      }

      headingDeg = corrigir_heading(headingDeg);

      // --- Construir CSV completo con fecha/hora al inicio ---
      String fechaHora = "";
      fechaHora += gps.date.year();
      if (gps.date.month() < 10) fechaHora += "0";
      fechaHora += gps.date.month();
      if (gps.date.day() < 10) fechaHora += "0";
      fechaHora += gps.date.day();
      fechaHora += "_";
      if (gps.time.hour() < 10) fechaHora += "0";
      fechaHora += gps.time.hour();
      if (gps.time.minute() < 10) fechaHora += "0";
      fechaHora += gps.time.minute();
      if (gps.time.second() < 10) fechaHora += "0";
      fechaHora += gps.time.second();

      String datos = fechaHora + "," +
                     String(gps.location.lat(), 6) + "," +
                     String(gps.location.lng(), 6) + "," +
                     String(gps.altitude.meters(), 2) + "," +
                     String(gps.speed.kmph(), 2) + "," +
                     String(headingDeg, 2) + "," +
                     String(gps.satellites.value());

      Serial3.println(datos);
      Serial.print(">> Enviado: ");
      Serial.println(datos);
    } else {
      Serial.println("GPS aún sin fix...");
    }
  }
}
