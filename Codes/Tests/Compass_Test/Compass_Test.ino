#include <TinyGPSPlus.h>
#include <Wire.h>

// ==================== GPS ====================
TinyGPSPlus gps;

// ==================== Brújula HMC5883L ====================
#define HMC5883L_ADDRESS 0x1E

// ==================== Tabla de calibración ====================
#define NUM_PONTOS 11
double tabela_leitura[NUM_PONTOS] = {0, 114.1, 148.4, 174.3, 197.1, 219.2, 240.3, 260.6, 280.4, 301.4, 360};
double tabela_real[NUM_PONTOS]    = {360, 300,   270,   240,   210,   180,   150,   120,    90,    60, 0};

// ==================== Filtro pasa-bajo ====================
double headingFiltrado = 0;  // valor inicial
const double alpha = 0.5;    // 0.1 = más suave, 0.5 = más rápido

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);   // GPS Ublox TX -> RX1 (pin 19 Mega)

  Wire.begin();          // I2C brújula

  // Inicialización HMC5883L
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x00); Wire.write(0x70); Wire.endTransmission(); // 8 muestras, 15Hz
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x01); Wire.write(0x20); Wire.endTransmission(); // Ganancia
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x02); Wire.write(0x00); Wire.endTransmission(); // Modo continuo

  Serial.println("Iniciando GPS + Brújula + Interpolación + Filtro...");
}

// ==================== Loop ====================
void loop() {
  // ---- Lectura GPS ----
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // ---- Lectura brújula ----
  int16_t x, y, z;
  leerBrujula(x, y, z);
  double headingLeido = atan2((double)y, (double)x) * 180.0 / PI;
  if (headingLeido < 0) headingLeido += 360.0;

  // ---- Corrección con interpolación ----
  double headingCorregido = corrigeHeading(headingLeido);

  // ---- Filtro pasa-bajo ----
  headingFiltrado = alpha * headingCorregido + (1.0 - alpha) * headingFiltrado;

  // ---- Prints ----
  if (gps.location.isValid()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lng: ");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("Lat: --- | Lng: ---");
  }

  Serial.print(" | Heading leido: ");
  Serial.print(headingLeido, 2);
  Serial.print("° | Heading corregido: ");
  Serial.print(headingCorregido, 2);
  Serial.print("° | Heading filtrado: ");
  Serial.print(headingFiltrado, 2);
  Serial.println("°");

  delay(200);
}

// ==================== Funciones ====================
void leerBrujula(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03); // Registro de datos
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L_ADDRESS, 6);
  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
}

// Interpolación lineal con circularidad
double corrigeHeading(double valor) {
  for (int i = 0; i < NUM_PONTOS - 1; i++) {
    if (valor >= tabela_leitura[i] && valor <= tabela_leitura[i + 1]) {
      double t = (valor - tabela_leitura[i]) / (tabela_leitura[i + 1] - tabela_leitura[i]);
      return fmod(tabela_real[i] + t * (tabela_real[i + 1] - tabela_real[i]) + 360.0, 360.0);
    }
  }

  // Manejo circular (último <-> primero +360)
  double ultimo = tabela_leitura[NUM_PONTOS - 1];
  double primero = tabela_leitura[0] + 360.0;
  if (valor >= ultimo && valor <= 360.0) {
    double t = (valor - ultimo) / (primero - ultimo);
    double real = tabela_real[NUM_PONTOS - 1] + t * ((tabela_real[0] + 360.0) - tabela_real[NUM_PONTOS - 1]);
    return fmod(real + 360.0, 360.0);
  }
  if (valor < tabela_leitura[0]) {
    double t = (valor + 360.0 - ultimo) / (primero - ultimo);
    double real = tabela_real[NUM_PONTOS - 1] + t * ((tabela_real[0] + 360.0) - tabela_real[NUM_PONTOS - 1]);
    return fmod(real + 360.0, 360.0);
  }

  return valor; // fallback
}
