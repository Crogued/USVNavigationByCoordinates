/**
 * Sistema de Navegação USV Simplificado
 * Controlo PID simples: só distancia perpendicular ao vetor
 * Sem funções complicadas - Só setup() y loop()
 * + FILTRO PASSA-BAIXO para valores PWM dos motores
 * + 3 COORDENADAS FIXAS PREDEFINIDAS
 * 
 * LÓGICA:
 * 1- Cheguei ao destino? (distância < m) → Mudar ponto
 * 2- Estou alinhado? (ângulo < a_min) → Se NÃO: rodar
 * 3- Se estiver alinhado → Aplicar PID para manter no vetor
 * 4- Filtrar valores PWM (80% atual, 20% anterior)
 */

#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>

// ========================================
//       PINOS E CONFIGURAÇÃO BÁSICA
// ========================================

#define PINO_CE 9
#define PINO_CSN 8
#define PINO_MOTOR_A 6  // Motor Direito
#define PINO_MOTOR_B 7  // Motor Esquerdo
#define PINO_LED_AMARELO 47
#define PINO_LED_VERDE 46

#define HMC5883L_ADDRESS 0x1E

#define PWM_CENTRO 1500
#define PWM_MIN 1000
#define PWM_MAX 2000

// ========================================
//    PARÂMETROS DE CONTROLO (EDITAVEIS)
// ========================================

// Distancia onde consideramos que chegamos ao destino (metros)
#define DISTANCIA_CHEGADA 5.0

// Ângulo onde consideramos que já estamos alinhados (graus)
#define ANGULO_ALINHADO 40.0

// Ganhos PID (só P e D, I=0)
#define KP 5.0
#define KD 2.0

// Velocidade base de avanço (PWM offset desde centro)
#define VELOCIDADE_BASE 300

// Velocidade de rotação quando não estamos alinhados
#define VELOCIDADE_VIRAR 250

// Filtro passa-baixo para PWM dos motores
#define ALPHA_PWM 0.8  // 80% valor atual, 20% valor anterior

// ========================================
//    COORDENADAS FIXAS (3 PONTOS)
// ========================================

#define NUM_VERTICES 3

// PONTO 0
#define LAT_P0 38.690896
#define LON_P0 -9.295136

// PONTO 1
#define LAT_P1 38.690644
#define LON_P1 -9.294929

// PONTO 2
#define LAT_P2 38.690720
#define LON_P2 -9.295220


// ========================================
//             COMANDOS RF24
// ========================================

#define MODO_MANUAL 0
#define MODO_AUTOMATICO 1
#define CMD_MOTOR_A 1
#define CMD_MOTOR_B 2
#define CMD_MODO 4
#define VAL_MODO_MANUAL 100
#define VAL_MODO_AUTO 200

// ========================================
//           VARIÁVEIS GLOBAIS
// ========================================

RF24 radio(PINO_CE, PINO_CSN);
Servo motorA, motorB;
TinyGPSPlus gps;
const byte endereco_rf24[6] = "00001";

int modo = MODO_MANUAL;
int modo_anterior = MODO_MANUAL;  // Para controlar prints de mudança de modo

// Vértices do polígono [lat, lon]
double vertices_lat[NUM_VERTICES];
double vertices_lon[NUM_VERTICES];

// Índices de navegação
int ponto_atual = 0;      // A
int ponto_destino = 1;     // B

// Posição atual do USV
double USV_lat = 0;
double USV_lon = 0;
double heading = 0;        // Da bússola

// Variáveis de controlo
double distancia_destino = 0;
double angulo_alinhado = 0;
double angulo_vetor = 0;   // Bearing objetivo
double erro_perpendicular = 0;
double erro_anterior = 0;

// PWM dos motores (valores calculados)
int pwm_motor_a = PWM_CENTRO;
int pwm_motor_b = PWM_CENTRO;

// PWM dos motores (valores filtrados)
double pwm_motor_a_filtrado = PWM_CENTRO;
double pwm_motor_b_filtrado = PWM_CENTRO;

// Bússola - filtro e calibração
double heading_filtrado = 0;
const double ALPHA_FILTRO = 0.5;

// Tabela de calibração bússola (11 pontos)
const double tabela_leitura[11] = {0, 114.1, 148.4, 174.3, 197.1, 219.2, 240.3, 260.6, 280.4, 301.4, 360};
const double tabela_real[11] = {360, 300, 270, 240, 210, 180, 150, 120, 90, 60, 0};

// ========================================
//                SETUP
// ========================================

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);  // GPS
  Serial3.begin(9600);  // Nano
  
  // Rádio RF24
  radio.begin();
  radio.openReadingPipe(0, endereco_rf24);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  
  // Bússola I2C
  Wire.begin();
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x01);
  Wire.write(0x20);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Motores
  motorA.attach(PINO_MOTOR_A);
  motorB.attach(PINO_MOTOR_B);
  motorA.writeMicroseconds(PWM_CENTRO);
  motorB.writeMicroseconds(PWM_CENTRO);
  
  // LEDs
  pinMode(PINO_LED_AMARELO, OUTPUT);
  pinMode(PINO_LED_VERDE, OUTPUT);
  digitalWrite(PINO_LED_AMARELO, HIGH);
  digitalWrite(PINO_LED_VERDE, LOW);
  
  // ========================================
  //    CARREGAR COORDENADAS FIXAS
  // ========================================
  
  vertices_lat[0] = LAT_P0;
  vertices_lon[0] = LON_P0;
  
  vertices_lat[1] = LAT_P1;
  vertices_lon[1] = LON_P1;
  
  vertices_lat[2] = LAT_P2;
  vertices_lon[2] = LON_P2;
  
  // ========================================
  //          PRINT INICIAL MELHORADO
  // ========================================
  
  Serial.println("\n========================================");
  Serial.println("   USV NAVEGAÇÃO SIMPLIFICADA");
  Serial.println("   + 3 Coordenadas Fixas");
  Serial.println("   + Filtro Passa-Baixo PWM");
  Serial.println("========================================");
  Serial.print("Número de pontos: ");
  Serial.println(NUM_VERTICES);
  Serial.print("Filtro PWM: ");
  Serial.print((int)(ALPHA_PWM * 100));
  Serial.print("% atual, ");
  Serial.print((int)((1.0 - ALPHA_PWM) * 100));
  Serial.println("% anterior");
  Serial.println("========================================");
  
  // MOSTRAR COORDENADAS E DISTÂNCIAS
  Serial.println("\n>>> COORDENADAS DOS PONTOS:");
  Serial.println("P# | Latitude   | Longitude  | Dist.Próx");
  Serial.println("---|------------|------------|----------");
  
  for (int i = 0; i < NUM_VERTICES; i++) {
    Serial.print("P");
    Serial.print(i);
    Serial.print(" | ");
    Serial.print(vertices_lat[i], 6);
    Serial.print(" | ");
    Serial.print(vertices_lon[i], 6);
    Serial.print(" | ");
    
    // Calcular distância até ao próximo ponto
    int proximo = (i + 1) % NUM_VERTICES;
    double delta_lat = (vertices_lat[proximo] - vertices_lat[i]) * PI / 180.0;
    double delta_lon = (vertices_lon[proximo] - vertices_lon[i]) * PI / 180.0;
    double a_dist = sin(delta_lat/2) * sin(delta_lat/2) + 
                    cos(vertices_lat[i] * PI / 180.0) * cos(vertices_lat[proximo] * PI / 180.0) * 
                    sin(delta_lon/2) * sin(delta_lon/2);
    double c_dist = 2 * atan2(sqrt(a_dist), sqrt(1-a_dist));
    double dist = 6371000.0 * c_dist;
    
    Serial.print(dist, 1);
    Serial.println("m");
  }
  
  Serial.println("========================================\n");
  
  delay(2000);
}

// ========================================
//                 LOOP
// ========================================

void loop() {
  
  // ========================================
  //              1. LER GPS
  // ========================================
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
  
  if (gps.location.isValid()) {
    USV_lat = gps.location.lat();
    USV_lon = gps.location.lng();
  }
  
  // ========================================
  //            2. LER BÚSSOLA
  // ========================================
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDRESS, 6);
  
  if (Wire.available() == 6) {
    int16_t x = Wire.read() << 8 | Wire.read();
    int16_t z = Wire.read() << 8 | Wire.read();
    int16_t y = Wire.read() << 8 | Wire.read();
    
    // Calcular heading raw
    double heading_raw = atan2((double)y, (double)x) * 180.0 / PI;
    if (heading_raw < 0) heading_raw += 360.0;
    
    // Corrigir com tabela de calibração (interpolação linear)
    double heading_corrigido = heading_raw;
    for (int i = 0; i < 10; i++) {
      if (heading_raw >= tabela_leitura[i] && heading_raw <= tabela_leitura[i+1]) {
        double t = (heading_raw - tabela_leitura[i]) / (tabela_leitura[i+1] - tabela_leitura[i]);
        heading_corrigido = tabela_real[i] + t * (tabela_real[i+1] - tabela_real[i]);
        break;
      }
    }
    
    // Filtro passa-baixo
    heading_filtrado = ALPHA_FILTRO * heading_corrigido + (1.0 - ALPHA_FILTRO) * heading_filtrado;
    heading = heading_filtrado;
  }
  
  // ========================================
  //        3. PROCESSAR COMANDOS RF24
  // ========================================
  if (radio.available()) {
    uint16_t dados = 0;
    radio.read(&dados, sizeof(dados));
    
    int comando = dados / 10000;
    int valor = dados % (comando * 10000);
    
    
    //Serial.print("Comando: ");
    //Serial.println(comando);
    Serial.println("------------------");
    Serial.print("Valor: ");
    Serial.println(valor);
    Serial.println("------------------");
    

    if (comando == CMD_MODO) {
      if (valor == VAL_MODO_MANUAL) {
        modo = MODO_MANUAL;
        motorA.writeMicroseconds(PWM_CENTRO);
        motorB.writeMicroseconds(PWM_CENTRO);
        pwm_motor_a_filtrado = PWM_CENTRO;
        pwm_motor_b_filtrado = PWM_CENTRO;
        digitalWrite(PINO_LED_AMARELO, HIGH);
        digitalWrite(PINO_LED_VERDE, LOW);
        
        // Mostrar mensagem apenas se mudou de modo
        if (modo_anterior != MODO_MANUAL) {
          Serial.println("\n>>> MODO MANUAL ATIVADO <<<\n");
          modo_anterior = MODO_MANUAL;
        }
      }
      else if (valor == VAL_MODO_AUTO) {
        modo = MODO_AUTOMATICO;
        digitalWrite(PINO_LED_AMARELO, LOW);
        digitalWrite(PINO_LED_VERDE, HIGH);
        erro_anterior = 0;
        pwm_motor_a_filtrado = PWM_CENTRO;
        pwm_motor_b_filtrado = PWM_CENTRO;
        
        // Mostrar mensagem apenas se mudou de modo
        if (modo_anterior != MODO_AUTOMATICO) {
          Serial.println("\n>>> MODO AUTOMÁTICO ATIVADO <<<\n");
          modo_anterior = MODO_AUTOMATICO;
        }
      }
    }
    else if (modo == MODO_MANUAL) {
      valor = constrain(valor, PWM_MIN, PWM_MAX);
      if (comando == CMD_MOTOR_A) {
        motorA.writeMicroseconds(valor);
        Serial.print("Motor A: ");
        Serial.println(valor);
      }
      else if (comando == CMD_MOTOR_B) {
        motorB.writeMicroseconds(valor);
        Serial.print("Motor B: ");
        Serial.println(valor);
      }
    }
  }
  
  // ========================================
  //     4. MODO AUTOMÁTICO - NAVEGAÇÃO
  // ========================================
  
  if (modo == MODO_AUTOMATICO && gps.location.isValid()) {
    
    // Coordenadas do ponto atual (A) e destino (B)
    double lat_A = vertices_lat[ponto_atual];
    double lon_A = vertices_lon[ponto_atual];
    double lat_B = vertices_lat[ponto_destino];
    double lon_B = vertices_lon[ponto_destino];
    
    // -----------------------------------
    //  CALCULAR DISTÂNCIA AO DESTINO (d)
    // -----------------------------------
    double delta_lat = (lat_B - USV_lat) * PI / 180.0;
    double delta_lon = (lon_B - USV_lon) * PI / 180.0;
    double a_dist = sin(delta_lat/2) * sin(delta_lat/2) + 
                    cos(USV_lat * PI / 180.0) * cos(lat_B * PI / 180.0) * 
                    sin(delta_lon/2) * sin(delta_lon/2);
    double c_dist = 2 * atan2(sqrt(a_dist), sqrt(1-a_dist));
    distancia_destino = 6371000.0 * c_dist;  // metros
    
    // -----------------------------------
    //  CALCULAR ÂNGULO DE ALINHAMENTO (a)
    // -----------------------------------
    
    // Vetor u = A → B (bearing geográfico, 0° = Norte)
    double dLat = (lat_B - lat_A) * PI / 180.0;
    double dLon = (lon_B - lon_A) * PI / 180.0;
    double lat1 = lat_A * PI / 180.0;
    double lat2 = lat_B * PI / 180.0;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    angulo_vetor = atan2(y, x) * 180.0 / PI;
    if (angulo_vetor < 0) angulo_vetor += 360.0;
    
    // Diferença entre heading do USV e direção do vetor
    double diff_angulo = heading - angulo_vetor;
    while (diff_angulo > 180.0) diff_angulo -= 360.0;
    while (diff_angulo < -180.0) diff_angulo += 360.0;
    angulo_alinhado = abs(diff_angulo);
    
    // -----------------------------------
    //  CALCULAR ERRO PERPENDICULAR (E)
    // -----------------------------------
    
    // Projeção do USV sobre o vetor A→B
    double cos_lat_ref = cos(lat_A * PI / 180.0);
    double USV_x = (USV_lon - lon_A) * cos_lat_ref * 111000.0;
    double USV_y = (USV_lat - lat_A) * 111000.0;
    double v_x = (lon_B - lon_A) * cos_lat_ref * 111000.0;
    double v_y = (lat_B - lat_A) * 111000.0;
    
    // Produto vetorial para determinar lado (+ = direita, - = esquerda)
    double cross = USV_x * v_y - USV_y * v_x;
    
    // Distância perpendicular
    double norma_vetor = sqrt(v_x * v_x + v_y * v_y);
    double dist_perp = abs(cross) / norma_vetor;
    
    // Erro com sinal
    erro_perpendicular = (cross > 0) ? dist_perp : -dist_perp;
    
    // ========================================
    //      FASE 1: CHEGÁMOS AO DESTINO?
    // ========================================
    
    if (distancia_destino < DISTANCIA_CHEGADA) {
      // MUDAR PARA O PRÓXIMO PONTO!
      
      // PRINT COMPLETO - FASE CHEGADA
      Serial.print("FASE:CHEGADA");
      Serial.print(" | GPS:");
      Serial.print(USV_lat, 6);
      Serial.print(",");
      Serial.print(USV_lon, 6);
      Serial.print(" | Vec:P");
      Serial.print(ponto_atual);
      Serial.print("→P");
      Serial.print(ponto_destino);
      Serial.print(" | D.Dest:");
      Serial.print(distancia_destino, 1);
      Serial.print("m | E.Perp:");
      Serial.print(erro_perpendicular, 2);
      Serial.print("m | Head:");
      Serial.print(heading, 1);
      Serial.print("° | Bear:");
      Serial.print(angulo_vetor, 1);
      Serial.print("° | Err.Ang:");
      Serial.print(angulo_alinhado, 1);
      Serial.print("° | PWM A:");
      Serial.print((int)pwm_motor_a_filtrado);
      Serial.print(" B:");
      Serial.println((int)pwm_motor_b_filtrado);
      
      ponto_atual = ponto_destino;
      ponto_destino = (ponto_destino + 1) % NUM_VERTICES;
      
      Serial.print(">>> NOVO DESTINO: P");
      Serial.print(ponto_atual);
      Serial.print(" → P");
      Serial.println(ponto_destino);
      Serial.println();
      
      erro_anterior = 0;
    }
    
    // ========================================
    //      FASE 2: ESTAMOS ALINHADOS?
    // ========================================
    
    else if (angulo_alinhado > ANGULO_ALINHADO) {
      // NÃO ALINHADO - RODAR
      
      // Determinar direção de rotação (menor ângulo)
      double diff = heading - angulo_vetor;
      
      while (diff > 180.0) diff -= 360.0;
      while (diff < -180.0) diff += 360.0;
      
      if (diff > 0) {
        // Rodar para a esquerda (motor B mais rápido)
        pwm_motor_a = PWM_CENTRO + VELOCIDADE_VIRAR;
        pwm_motor_b = PWM_CENTRO - VELOCIDADE_VIRAR;
      } else {
        // Rodar para a direita (motor A mais rápido)
        pwm_motor_a = PWM_CENTRO - VELOCIDADE_VIRAR;
        pwm_motor_b = PWM_CENTRO + VELOCIDADE_VIRAR;
      }

      pwm_motor_a_filtrado = ALPHA_PWM * pwm_motor_a + (1.0 - ALPHA_PWM) * pwm_motor_a_filtrado;
      pwm_motor_b_filtrado = ALPHA_PWM * pwm_motor_b + (1.0 - ALPHA_PWM) * pwm_motor_b_filtrado;

      // Escrever o valor FILTRADO
      motorA.writeMicroseconds((int)pwm_motor_a_filtrado);
      motorB.writeMicroseconds((int)pwm_motor_b_filtrado);
      
      // PRINT COMPLETO - FASE RODAR
      Serial.print("FASE:RODAR");
      Serial.print(" | GPS:");
      Serial.print(USV_lat, 6);
      Serial.print(",");
      Serial.print(USV_lon, 6);
      Serial.print(" | Vec:P");
      Serial.print(ponto_atual);
      Serial.print("→P");
      Serial.print(ponto_destino);
      Serial.print(" | D.Dest:");
      Serial.print(distancia_destino, 1);
      Serial.print("m | E.Perp:");
      Serial.print(erro_perpendicular, 2);
      Serial.print("m | Head:");
      Serial.print(heading, 1);
      Serial.print("° | Bear:");
      Serial.print(angulo_vetor, 1);
      Serial.print("° | Err.Ang:");
      Serial.print(angulo_alinhado, 1);
      Serial.print("° | PWM A:");
      Serial.print(pwm_motor_a_filtrado);
      Serial.print(" B:");
      Serial.println(pwm_motor_b_filtrado);
    }
    
    // ========================================
    //     FASE 3: ALINHADO - APLICAR PID
    // ========================================
    
    else {
      // Calcular PID (só P e D)
      double P = KP * erro_perpendicular;
      double D = KD * (erro_perpendicular - erro_anterior);
      double PD = P + D;
      
      // Aplicar aos motores
      pwm_motor_a = PWM_CENTRO + VELOCIDADE_BASE + (int)PD;
      pwm_motor_b = PWM_CENTRO + VELOCIDADE_BASE - (int)PD;
      
      // Limitar PWM
      pwm_motor_a = constrain(pwm_motor_a, PWM_MIN, PWM_MAX);
      pwm_motor_b = constrain(pwm_motor_b, PWM_MIN, PWM_MAX);
      
      // ========================================
      //      APLICAR FILTRO PASSA-BAIXO PWM
      // ========================================
      pwm_motor_a_filtrado = ALPHA_PWM * pwm_motor_a + (1.0 - ALPHA_PWM) * pwm_motor_a_filtrado;
      pwm_motor_b_filtrado = ALPHA_PWM * pwm_motor_b + (1.0 - ALPHA_PWM) * pwm_motor_b_filtrado;
      
      motorA.writeMicroseconds((int)pwm_motor_a_filtrado);
      motorB.writeMicroseconds((int)pwm_motor_b_filtrado);
      
      // Guardar erro para próxima iteração
      erro_anterior = erro_perpendicular;
      
      // PRINT COMPLETO - FASE NAVEGAR
      Serial.print("FASE:NAVEGAR");
      Serial.print(" | GPS:");
      Serial.print(USV_lat, 6);
      Serial.print(",");
      Serial.print(USV_lon, 6);
      Serial.print(" | Vec:P");
      Serial.print(ponto_atual);
      Serial.print("→P");
      Serial.print(ponto_destino);
      Serial.print(" | D.Dest:");
      Serial.print(distancia_destino, 1);
      Serial.print("m | E.Perp:");
      Serial.print(erro_perpendicular, 2);
      Serial.print("m | Head:");
      Serial.print(heading, 1);
      Serial.print("° | Bear:");
      Serial.print(angulo_vetor, 1);
      Serial.print("° | Err.Ang:");
      Serial.print(angulo_alinhado, 1);
      Serial.print("° | PWM A:");
      Serial.print((int)pwm_motor_a_filtrado);
      Serial.print(" B:");
      Serial.println((int)pwm_motor_b_filtrado);
    }
    
    // ========================================
    //          ENVIAR DADOS AO NANO
    // ========================================
    
    if (gps.date.isValid() && gps.time.isValid()) {
      String timestamp = "";
      timestamp += gps.date.year();
      if (gps.date.month() < 10) timestamp += "0";
      timestamp += gps.date.month();
      if (gps.date.day() < 10) timestamp += "0";
      timestamp += gps.date.day();
      timestamp += "_";
      if (gps.time.hour() < 10) timestamp += "0";
      timestamp += gps.time.hour();
      if (gps.time.minute() < 10) timestamp += "0";
      timestamp += gps.time.minute();
      if (gps.time.second() < 10) timestamp += "0";
      timestamp += gps.time.second();
      
      String dados = timestamp + "," +
                     String(USV_lat, 6) + "," +
                     String(USV_lon, 6) + "," +
                     String(ponto_atual) + "," +
                     String(ponto_destino) + "," +
                     String(distancia_destino, 2) + "," +
                     String(erro_perpendicular, 2) + "," +
                     String(heading, 1) + "," +
                     String(angulo_vetor, 1) + "," +
                     String(angulo_alinhado) + "," +
                     String((int)pwm_motor_a_filtrado) + "," +
                     String((int)pwm_motor_b_filtrado);
      
      Serial3.println(dados);
    }
  }
  
  delay(50);
}