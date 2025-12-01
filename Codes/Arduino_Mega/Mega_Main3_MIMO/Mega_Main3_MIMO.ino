/**
 * Sistema de Navegação por Vectores para USV com Controlo de Heading
 * Versão com Brújula HMC5883L integrada
 * 
 * NOVIDADES NESTA VERSÃO:
 * - Controlo de heading com brújula HMC5883L
 * - Navegação em duas fases: APROXIMAÇÃO e SEGUIMENTO
 * - Capacidade de "crab" para contrarrestar correntes
 * - PID completo para controlo de rotação (I e D configuráveis)
 * - Interpolação e calibração de heading
 * - Filtro pasa-bajo para suavizar leituras
 * 
 * AUTOR: Sistema USV v4.0 - Navegação por Vectores com Heading
 * DATA: 2025
 */

#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>

// ========================================
//       CONSTANTES DE CONFIGURAÇÃO
// ========================================

// Definições dos pinos do hardware
#define PINO_CE 9
#define PINO_CSN 8
#define PINO_MOTOR_A 7 // Motor Direito
#define PINO_MOTOR_B 6 // Motor Esquerdo
#define PINO_LED_AMARELO 47
#define PINO_LED_VERDE 46

// Endereço I2C da brújula
#define HMC5883L_ADDRESS 0x1E

// Modos de operação
#define MODO_MANUAL 0
#define MODO_AUTOMATICO 1

// Comandos RF24
#define CMD_MOTOR_A 1
#define CMD_MOTOR_B 2
#define CMD_MODO 4
#define VAL_MODO_MANUAL 100
#define VAL_MODO_AUTO 200

// Parâmetros PWM dos motores
#define PWM_CENTRO 1500
#define PWM_MIN 1200
#define PWM_MAX 1800

// ========================================
//   CONFIGURAÇÕES DO POLÍGONO (EDITÁVEIS)
// ========================================

#define NUM_VERTICES 3
#define DISTANCIA_ENTRE_VERTICES 15.0
#define RAIO_TOLERANCIA_VERTICE 2.0

// **NOVO**: Distância para mudar de fase APROXIMAÇÃO para SEGUIMENTO
#define DISTANCIA_TRANSICAO_FASE 2.0  // metros

// ========================================
//   PARÂMETROS PID (EDITÁVEIS)
// ========================================

// PID para controlo de velocidade (distância)
#define KP_AVANCO 5.0
#define KI_AVANCO 0.0  // Desactivado inicialmente
#define KD_AVANCO 0.0  // Desactivado inicialmente

// PID para controlo de rotação (heading)
#define KP_HEADING 3.0   // Ganho proporcional para erro de heading
#define KI_HEADING 0.0   // Desactivado inicialmente (testar depois)
#define KD_HEADING 0.0   // Desactivado inicialmente (testar depois)

// Limites para integral (anti-windup)
#define LIMITE_INTEGRAL_HEADING 50.0
#define LIMITE_INTEGRAL_AVANCO 20.0

// ========================================
//   CONFIGURAÇÕES DA BRÚJULA
// ========================================

// Tabela de calibração (interpolação linear)
#define NUM_PONTOS_CALIBRACAO 11
const double tabela_leitura[NUM_PONTOS_CALIBRACAO] = {0, 114.1, 148.4, 174.3, 197.1, 219.2, 240.3, 260.6, 280.4, 301.4, 360};
const double tabela_real[NUM_PONTOS_CALIBRACAO]    = {360, 300,   270,   240,   210,   180,   150,   120,    90,    60, 0};

// Filtro pasa-bajo para heading
#define ALPHA_FILTRO_HEADING 0.5

// ========================================
//   OUTRAS CONFIGURAÇÕES
// ========================================

#define MAX_VERTICES 12
#define INTERVALO_PISCADA_MS 500
#define INTERVALO_ENVIO_DADOS_MS 500
#define ENDERECO_RF24 "00001"
#define GPS_BAUD 9600
#define NANO_BAUD 9600

// ========================================
//   ENUMERAÇÕES E ESTRUTURAS
// ========================================

/**
 * Fases de navegação
 */
enum FaseNavegacao {
  FASE_APROXIMACAO,   // Aproximar-se ao vector mais próximo
  FASE_SEGUIMENTO     // Seguir o vector até ao vértice
};

struct Ponto {
  double lat;
  double lon;
};

struct EstadoVector {
  int vertice_origem;
  int vertice_destino;
  Ponto ponto_origem;
  Ponto ponto_destino;
  Ponto ponto_mais_proximo;
  double distancia_ao_destino;
  double distancia_ao_vector;
  double progresso_vector;
};

/**
 * **NOVO**: Estrutura para dados da brújula
 */
struct DadosBrujula {
  double heading_raw;           // Heading lido directamente
  double heading_corrigido;     // Heading após interpolação
  double heading_filtrado;      // Heading após filtro pasa-bajo
  int16_t mag_x, mag_y, mag_z;  // Valores brutos do magnetómetro
};

/**
 * **NOVO**: Estrutura para controlo PID de heading
 */
struct PIDHeading {
  double erro_actual;
  double erro_anterior;
  double integral;
  double derivada;
  double saida;
};

/**
 * **NOVO**: Estrutura para controlo PID de avanço
 */
struct PIDAvanco {
  double erro_actual;
  double erro_anterior;
  double integral;
  double derivada;
  double saida;
};

struct EstadoNavegacao {
  double latitude_actual;
  double longitude_actual;
  double velocidade_kmh;
  EstadoVector vector_actual;
  DadosBrujula brujula;         // **NOVO**
  FaseNavegacao fase;           // **NOVO**
  double bearing_objetivo;      // **NOVO**: Bearing desejado (graus)
};

struct ControlosPID {
  double erro_avanco;
  double erro_vector;
  double erro_heading;          // **NOVO**
  int sinal_avanco;
  int sinal_rotacao;
  int pwm_motor_a;
  int pwm_motor_b;
  PIDHeading pid_heading;       // **NOVO**
  PIDAvanco pid_avanco;         // **NOVO**
};

// ========================================
//             VARIÁVEIS GLOBAIS
// ========================================

RF24 radio(PINO_CE, PINO_CSN);
Servo motorA, motorB;
TinyGPSPlus gps;

const byte ENDERECO_RF24_BYTES[6] = ENDERECO_RF24;

int modo_operacao = MODO_MANUAL;
bool erro_detectado = false;
bool sistema_iniciado = false;

unsigned long ultima_piscada_led = 0;
unsigned long ultimo_envio_dados = 0;
bool estado_led_erro = false;

EstadoNavegacao estado_nav = {0};
ControlosPID controlos = {0};

struct PwmMotorStruct {
  int motor_A;
  int motor_B;
} pwm_motor;

Ponto vertices[MAX_VERTICES];
int total_vertices = 0;

// Coordenada base do polígono
const double lat0 = 38.691759;
const double lon0 = -9.299067;

// ========================================
//           PROTÓTIPOS DE FUNÇÕES
// ========================================

// Funções de inicialização
void inicializar_comunicacoes();
bool inicializar_radio();
void inicializar_motores();
void inicializar_leds();
bool inicializar_brujula();

// Controlo de motores e LEDs
void parar_motores();
void definir_velocidade_motores(int pwm_a, int pwm_b);
void activar_led_modo_manual();
void activar_led_modo_automatico();
void piscar_led_erro();
void actualizar_leds();

// Comunicação RF24
bool processar_mudanca_modo(int valor);
void processar_comando_manual(int comando, int valor);
void processar_comandos_rf24();

// **NOVO**: Funções da brújula
void ler_brujula(int16_t &x, int16_t &y, int16_t &z);
double calcular_heading_raw(int16_t x, int16_t y);
double corrigir_heading_interpolacao(double heading_raw);
void actualizar_brujula();

// Navegação GPS e vectores
double calcular_distancia_haversine(double lat1, double lon1, double lat2, double lon2);
double calcular_bearing(double lat1, double lon1, double lat2, double lon2);
void calcular_ponto_mais_proximo_vector(const Ponto& barco, const Ponto& v1, const Ponto& v2, 
                                       Ponto* ponto_proximo, double* distancia_vector, double* progresso);
void actualizar_estado_vector(EstadoVector* vector_estado, const Ponto& posicao_barco);

// **NOVO**: Controlo PID com heading
void calcular_controlos_pid_heading(EstadoNavegacao* estado, ControlosPID* controlos);
double normalizar_erro_angulo(double erro);

void imprimir_estado_navegacao_vector();
void actualizar_gps();

// Polígono
void gerar_poligono_regular(double baseLat, double baseLon, int num_vertices, double distancia_metros);
void imprimir_vertices_poligono();
void executar_navegacao_por_vectores();
bool verificar_vertice_atingido(const Ponto& posicao_barco, const Ponto& vertice_destino);

// Comunicação Nano
void enviar_dados_nano();

// Utilitários
double graus_para_radianos(double graus);
double radianos_para_graus(double radianos);
Ponto deslocar_coordenada_metros(double lat, double lon, double distancia_m, double angulo_graus);

// ========================================
//                 SETUP
// ========================================

void setup() {
  inicializar_comunicacoes();
  
  if (!inicializar_radio()) {
    Serial.println(">>> ATENÇÃO: Rádio não inicializado correctamente.");
  }
  
  // **NOVO**: Inicializar brújula
  if (!inicializar_brujula()) {
    Serial.println(">>> ATENÇÃO: Brújula não inicializada correctamente.");
  }
  
  inicializar_motores();
  inicializar_leds();

  Serial.println("\n>>> A gerar polígono de navegação...");
  gerar_poligono_regular(lat0, lon0, NUM_VERTICES, DISTANCIA_ENTRE_VERTICES);
  imprimir_vertices_poligono();

  // Inicializar estado do vector
  estado_nav.vector_actual.vertice_origem = 0;
  estado_nav.vector_actual.vertice_destino = 1;
  estado_nav.vector_actual.ponto_origem = vertices[0];
  estado_nav.vector_actual.ponto_destino = vertices[1];
  estado_nav.fase = FASE_APROXIMACAO;  // **NOVO**

  delay(2000);
  sistema_iniciado = true;

  Serial.println("\n=================================================");
  Serial.println(">>> SISTEMA USV COM CONTROLO DE HEADING INICIADO <<<");
  Serial.print(">>> Polígono: ");
  Serial.print(NUM_VERTICES);
  Serial.print(" vértices, ");
  Serial.print((int)DISTANCIA_ENTRE_VERTICES);
  Serial.println(" metros entre vértices <<<");
  Serial.println(">>> Controlo de heading com brújula HMC5883L <<<");
  Serial.println("=================================================");
  Serial.println("Mega pronto, a aguardar fix GPS...\n");
}

// ========================================
//      FUNÇÕES DE INICIALIZAÇÃO
// ========================================

void inicializar_comunicacoes() {
  Serial.begin(9600);
  Serial2.begin(GPS_BAUD);
  Serial3.begin(NANO_BAUD);
  Serial.println(">>> Comunicações inicializadas");
}

bool inicializar_radio() {
  if (!radio.begin()) {
    Serial.println(">>> ERRO: Falha na inicialização do rádio!");
    return false;
  }
  
  radio.openReadingPipe(0, ENDERECO_RF24_BYTES);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  
  Serial.println(">>> Rádio RF24 inicializado");
  return true;
}

/**
 * **NOVO**: Inicializa a brújula HMC5883L via I2C
 */
bool inicializar_brujula() {
  Wire.begin();
  
  // Configurar HMC5883L
  // Registo 0x00: 8 amostras, 15Hz
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x70);
  if (Wire.endTransmission() != 0) {
    Serial.println(">>> ERRO: Brújula não responde!");
    return false;
  }
  
  // Registo 0x01: Ganho
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x01);
  Wire.write(0x20);
  Wire.endTransmission();
  
  // Registo 0x02: Modo contínuo
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Serial.println(">>> Brújula HMC5883L inicializada");
  
  // Inicializar filtro com leitura inicial
  int16_t x, y, z;
  ler_brujula(x, y, z);
  double heading_inicial = calcular_heading_raw(x, y);
  estado_nav.brujula.heading_filtrado = corrigir_heading_interpolacao(heading_inicial);
  
  return true;
}

void inicializar_motores() {
  motorA.attach(PINO_MOTOR_A);
  motorB.attach(PINO_MOTOR_B);
  parar_motores();
  Serial.println(">>> Motores inicializados e parados");
}

void inicializar_leds() {
  pinMode(PINO_LED_AMARELO, OUTPUT);
  pinMode(PINO_LED_VERDE, OUTPUT);
  activar_led_modo_manual();
  Serial.println(">>> LEDs configurados");
}

// ========================================
//      CONTROLO MOTORES E LEDS
// ========================================

void parar_motores() {
  motorA.writeMicroseconds(PWM_CENTRO);
  motorB.writeMicroseconds(PWM_CENTRO);
  pwm_motor.motor_A = PWM_CENTRO;
  pwm_motor.motor_B = PWM_CENTRO;
}

void definir_velocidade_motores(int pwm_a, int pwm_b) {
  pwm_a = constrain(pwm_a, PWM_MIN, PWM_MAX);
  pwm_b = constrain(pwm_b, PWM_MIN, PWM_MAX);
  
  motorA.writeMicroseconds(pwm_a);
  motorB.writeMicroseconds(pwm_b);
  
  pwm_motor.motor_A = pwm_a;
  pwm_motor.motor_B = pwm_b;
}

void activar_led_modo_manual() {
  digitalWrite(PINO_LED_AMARELO, HIGH);
  digitalWrite(PINO_LED_VERDE, LOW);
}

void activar_led_modo_automatico() {
  digitalWrite(PINO_LED_AMARELO, LOW);
  digitalWrite(PINO_LED_VERDE, HIGH);
}

void piscar_led_erro() {
  unsigned long tempo_actual = millis();
  if (tempo_actual - ultima_piscada_led > INTERVALO_PISCADA_MS) {
    ultima_piscada_led = tempo_actual;
    estado_led_erro = !estado_led_erro;
    digitalWrite(PINO_LED_AMARELO, estado_led_erro ? HIGH : LOW);
    digitalWrite(PINO_LED_VERDE, LOW);
  }
}

void actualizar_leds() {
  if (erro_detectado) {
    piscar_led_erro();
  }
  else if (modo_operacao == MODO_MANUAL) {
    activar_led_modo_manual();
  }
  else {
    activar_led_modo_automatico();
  }
}

// ========================================
//      COMUNICAÇÃO RF24
// ========================================

bool processar_mudanca_modo(int valor) {
  int novo_modo = -1;
  
  if (valor == VAL_MODO_MANUAL) {
    novo_modo = MODO_MANUAL;
  }
  else if (valor == VAL_MODO_AUTO) {
    novo_modo = MODO_AUTOMATICO;
  }
  else {
    Serial.print(">>> ERRO: Valor de modo inválido recebido: ");
    Serial.println(valor);
    return false;
  }

  if (novo_modo != modo_operacao) {
    modo_operacao = novo_modo;
    erro_detectado = false;
    
    if (novo_modo == MODO_MANUAL) {
      parar_motores();
      Serial.println(">>> MODO MANUAL ACTIVADO");
    } else {
      Serial.println(">>> MODO AUTOMÁTICO ACTIVADO - Navegação por Vectores com Heading");
      estado_nav.vector_actual.vertice_origem = 0;
      estado_nav.vector_actual.vertice_destino = 1;
      estado_nav.vector_actual.ponto_origem = vertices[0];
      estado_nav.vector_actual.ponto_destino = vertices[1];
      estado_nav.fase = FASE_APROXIMACAO;
      
      // Reset PID
      controlos.pid_heading.integral = 0;
      controlos.pid_heading.erro_anterior = 0;
      controlos.pid_avanco.integral = 0;
      controlos.pid_avanco.erro_anterior = 0;
    }
  }
  return true;
}

void processar_comando_manual(int comando, int valor) {
  if (modo_operacao != MODO_MANUAL) {
    Serial.println(">>> AVISO: Comando manual recebido fora do modo manual!");
    return;
  }
  
  valor = constrain(valor, PWM_MIN, PWM_MAX);
  
  if (comando == CMD_MOTOR_A) {
    motorA.writeMicroseconds(valor);
    pwm_motor.motor_A = valor;
  }
  else if (comando == CMD_MOTOR_B) {
    motorB.writeMicroseconds(valor);
    pwm_motor.motor_B = valor;
  }
  else {
    Serial.println(">>> ERRO: Comando de motor inválido!");
    erro_detectado = true;
  }
}

void processar_comandos_rf24() {
  uint16_t dados_recebidos = 0;
  
  if (!radio.available()) return;
  
  radio.read(&dados_recebidos, sizeof(dados_recebidos));
  
  int comando = dados_recebidos / 10000;
  int valor = dados_recebidos % (comando * 10000);
  
  if (comando == CMD_MODO) {
    if (!processar_mudanca_modo(valor)) {
      erro_detectado = true;
    }
  }
  else if (comando == CMD_MOTOR_A || comando == CMD_MOTOR_B) {
    processar_comando_manual(comando, valor);
  }
  else {
    Serial.print(">>> ERRO: Comando RF24 inválido: ");
    Serial.println(comando);
    erro_detectado = true;
  }
}

// ========================================
//           FUNÇÕES DA BRÚJULA
// ========================================

/**
 * **NOVO**: Lê valores brutos do magnetómetro
 */
void ler_brujula(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03);  // Registo de dados
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L_ADDRESS, 6);
  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
}

/**
 * **NOVO**: Calcula heading bruto a partir dos valores X e Y
 */
double calcular_heading_raw(int16_t x, int16_t y) {
  double heading = atan2((double)y, (double)x) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

/**
 * **NOVO**: Corrige heading usando interpolação linear com a tabela de calibração
 */
double corrigir_heading_interpolacao(double valor) {
  // Interpolação entre pontos da tabela
  for (int i = 0; i < NUM_PONTOS_CALIBRACAO - 1; i++) {
    if (valor >= tabela_leitura[i] && valor <= tabela_leitura[i + 1]) {
      double t = (valor - tabela_leitura[i]) / (tabela_leitura[i + 1] - tabela_leitura[i]);
      return fmod(tabela_real[i] + t * (tabela_real[i + 1] - tabela_real[i]) + 360.0, 360.0);
    }
  }

  // Manejo circular (último <-> primeiro +360)
  double ultimo = tabela_leitura[NUM_PONTOS_CALIBRACAO - 1];
  double primero = tabela_leitura[0] + 360.0;
  
  if (valor >= ultimo && valor <= 360.0) {
    double t = (valor - ultimo) / (primero - ultimo);
    double real = tabela_real[NUM_PONTOS_CALIBRACAO - 1] + 
                  t * ((tabela_real[0] + 360.0) - tabela_real[NUM_PONTOS_CALIBRACAO - 1]);
    return fmod(real + 360.0, 360.0);
  }
  
  if (valor < tabela_leitura[0]) {
    double t = (valor + 360.0 - ultimo) / (primero - ultimo);
    double real = tabela_real[NUM_PONTOS_CALIBRACAO - 1] + 
                  t * ((tabela_real[0] + 360.0) - tabela_real[NUM_PONTOS_CALIBRACAO - 1]);
    return fmod(real + 360.0, 360.0);
  }

  return valor;
}

/**
 * **NOVO**: Actualiza todos os dados da brújula
 */
void actualizar_brujula() {
  // Ler valores brutos
  ler_brujula(estado_nav.brujula.mag_x, 
              estado_nav.brujula.mag_y, 
              estado_nav.brujula.mag_z);
  
  // Calcular heading bruto
  estado_nav.brujula.heading_raw = calcular_heading_raw(
    estado_nav.brujula.mag_x, 
    estado_nav.brujula.mag_y
  );
  
  // Corrigir com interpolação
  estado_nav.brujula.heading_corrigido = corrigir_heading_interpolacao(
    estado_nav.brujula.heading_raw
  );
  
  // Aplicar filtro pasa-bajo
  estado_nav.brujula.heading_filtrado = 
    ALPHA_FILTRO_HEADING * estado_nav.brujula.heading_corrigido + 
    (1.0 - ALPHA_FILTRO_HEADING) * estado_nav.brujula.heading_filtrado;
}

// ========================================
//         NAVEGAÇÃO POR VECTORES
// ========================================

double calcular_distancia_haversine(double lat1, double lon1, double lat2, double lon2) {
  const double RAIO_TERRA_M = 6371000.0;
  
  double delta_lat = graus_para_radianos(lat2 - lat1);
  double delta_lon = graus_para_radianos(lon2 - lon1);
  
  double a = sin(delta_lat/2) * sin(delta_lat/2) +
             cos(graus_para_radianos(lat1)) * cos(graus_para_radianos(lat2)) *
             sin(delta_lon/2) * sin(delta_lon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return RAIO_TERRA_M * c;
}

/**
 * **NOVO**: Calcula bearing (rumo) entre dois pontos GPS
 */
double calcular_bearing(double lat1, double lon1, double lat2, double lon2) {
  double lat1_rad = graus_para_radianos(lat1);
  double lat2_rad = graus_para_radianos(lat2);
  double delta_lon = graus_para_radianos(lon2 - lon1);
  
  double y = sin(delta_lon) * cos(lat2_rad);
  double x = cos(lat1_rad) * sin(lat2_rad) - 
             sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
  
  double bearing = atan2(y, x);
  bearing = radianos_para_graus(bearing);
  bearing = fmod(bearing + 360.0, 360.0);
  
  return bearing;
}

void calcular_ponto_mais_proximo_vector(const Ponto& barco, const Ponto& v1, const Ponto& v2, 
                                       Ponto* ponto_proximo, double* distancia_vector, double* progresso) {
  
  double lat_ref = v1.lat;
  double lon_ref = v1.lon;
  double cos_lat = cos(graus_para_radianos(lat_ref));
  
  double v1_x = 0.0;
  double v1_y = 0.0;
  double v2_x = (v2.lon - v1.lon) * cos_lat * 111000.0;
  double v2_y = (v2.lat - v1.lat) * 111000.0;
  double barco_x = (barco.lon - v1.lon) * cos_lat * 111000.0;
  double barco_y = (barco.lat - v1.lat) * 111000.0;
  
  double vector_x = v2_x - v1_x;
  double vector_y = v2_y - v1_y;
  
  double v1_barco_x = barco_x - v1_x;
  double v1_barco_y = barco_y - v1_y;
  
  double produto_escalar = v1_barco_x * vector_x + v1_barco_y * vector_y;
  double norma_vector_2 = vector_x * vector_x + vector_y * vector_y;
  
  double t = 0.0;
  if (norma_vector_2 > 1e-10) {
    t = produto_escalar / norma_vector_2;
  }
  
  t = constrain(t, 0.0, 1.0);
  
  double ponto_x = v1_x + t * vector_x;
  double ponto_y = v1_y + t * vector_y;
  
  ponto_proximo->lat = lat_ref + ponto_y / 111000.0;
  ponto_proximo->lon = lon_ref + ponto_x / (111000.0 * cos_lat);
  
  double dist_x = barco_x - ponto_x;
  double dist_y = barco_y - ponto_y;
  *distancia_vector = sqrt(dist_x * dist_x + dist_y * dist_y);
  
  *progresso = t;
}

void actualizar_estado_vector(EstadoVector* vector_estado, const Ponto& posicao_barco) {
  calcular_ponto_mais_proximo_vector(
    posicao_barco,
    vector_estado->ponto_origem,
    vector_estado->ponto_destino,
    &vector_estado->ponto_mais_proximo,
    &vector_estado->distancia_ao_vector,
    &vector_estado->progresso_vector
  );
  
  vector_estado->distancia_ao_destino = calcular_distancia_haversine(
    posicao_barco.lat, posicao_barco.lon,
    vector_estado->ponto_destino.lat, vector_estado->ponto_destino.lon
  );
}

/**
 * **NOVO**: Normaliza erro angular para estar entre -180° e +180°
 */
double normalizar_erro_angulo(double erro) {
  while (erro > 180.0) erro -= 360.0;
  while (erro < -180.0) erro += 360.0;
  return erro;
}

/**
 * **NOVO**: Calcula controlos PID com base em heading e fase de navegação
 * 
 * LÓGICA:
 * - FASE_APROXIMACAO: Ir direto ao ponto mais próximo no vector
 * - FASE_SEGUIMENTO: Seguir o vector, permitindo "crab" para contrarrestar correntes
 */
void calcular_controlos_pid_heading(EstadoNavegacao* estado, ControlosPID* controlos) {
  Ponto posicao_barco = {estado->latitude_actual, estado->longitude_actual};
  
  // Determinar objetivo baseado na fase
  Ponto objetivo;
  
  if (estado->fase == FASE_APROXIMACAO) {
    // Na aproximação: ir direto ao ponto mais próximo no vector
    objetivo = estado->vector_actual.ponto_mais_proximo;
  } else {
    // No seguimento: ir direto ao vértice destino
    objetivo = estado->vector_actual.ponto_destino;
  }
  
  // Calcular bearing desejado (direção para o objetivo)
  estado->bearing_objetivo = calcular_bearing(
    posicao_barco.lat, posicao_barco.lon,
    objetivo.lat, objetivo.lon
  );
  
  // ===== CONTROLO DE HEADING (ROTAÇÃO) =====
  
  // Erro de heading = diferença entre bearing desejado e heading actual
  controlos->erro_heading = estado->bearing_objetivo - estado->brujula.heading_filtrado;
  controlos->erro_heading = normalizar_erro_angulo(controlos->erro_heading);
  
  // PID para heading
  controlos->pid_heading.erro_actual = controlos->erro_heading;
  
  // Termo Proporcional
  double P_heading = KP_HEADING * controlos->pid_heading.erro_actual;
  
  // Termo Integral (com anti-windup)
  controlos->pid_heading.integral += controlos->pid_heading.erro_actual;
  controlos->pid_heading.integral = constrain(
    controlos->pid_heading.integral, 
    -LIMITE_INTEGRAL_HEADING, 
    LIMITE_INTEGRAL_HEADING
  );
  double I_heading = KI_HEADING * controlos->pid_heading.integral;
  
  // Termo Derivativo
  controlos->pid_heading.derivada = 
    controlos->pid_heading.erro_actual - controlos->pid_heading.erro_anterior;
  double D_heading = KD_HEADING * controlos->pid_heading.derivada;
  
  // Sinal de controlo de rotação
  controlos->pid_heading.saida = P_heading + I_heading + D_heading;
  controlos->sinal_rotacao = (int)controlos->pid_heading.saida;
  
  // Guardar erro para próxima iteração
  controlos->pid_heading.erro_anterior = controlos->pid_heading.erro_actual;
  
  // ===== CONTROLO DE AVANÇO (VELOCIDADE) =====
  
  // Na fase de aproximação: erro = distância ao vector
  // Na fase de seguimento: erro = distância ao vértice destino
  if (estado->fase == FASE_APROXIMACAO) {
    controlos->erro_avanco = estado->vector_actual.distancia_ao_vector;
  } else {
    controlos->erro_avanco = estado->vector_actual.distancia_ao_destino;
  }
  
  // PID para avanço
  controlos->pid_avanco.erro_actual = controlos->erro_avanco;
  
  // Termo Proporcional
  double P_avanco = KP_AVANCO * controlos->pid_avanco.erro_actual;
  
  // Termo Integral (com anti-windup)
  controlos->pid_avanco.integral += controlos->pid_avanco.erro_actual;
  controlos->pid_avanco.integral = constrain(
    controlos->pid_avanco.integral,
    -LIMITE_INTEGRAL_AVANCO,
    LIMITE_INTEGRAL_AVANCO
  );
  double I_avanco = KI_AVANCO * controlos->pid_avanco.integral;
  
  // Termo Derivativo
  controlos->pid_avanco.derivada = 
    controlos->pid_avanco.erro_actual - controlos->pid_avanco.erro_anterior;
  double D_avanco = KD_AVANCO * controlos->pid_avanco.derivada;
  
  // Sinal de controlo de avanço
  controlos->pid_avanco.saida = P_avanco + I_avanco + D_avanco;
  controlos->sinal_avanco = (int)controlos->pid_avanco.saida;
  
  // Garantir velocidade mínima para frente (evitar reversa)
  controlos->sinal_avanco = max(50, controlos->sinal_avanco);
  
  // Guardar erro para próxima iteração
  controlos->pid_avanco.erro_anterior = controlos->pid_avanco.erro_actual;
  
  // ===== COMBINAR SINAIS PARA PWM FINAL =====
  
  // Motor A (esquerdo): base + avanço - rotação
  // Motor B (direito): base + avanço + rotação
  controlos->pwm_motor_a = PWM_CENTRO + controlos->sinal_avanco - controlos->sinal_rotacao;
  controlos->pwm_motor_b = PWM_CENTRO + controlos->sinal_avanco + controlos->sinal_rotacao;
  
  // Limitar valores PWM
  controlos->pwm_motor_a = constrain(controlos->pwm_motor_a, PWM_MIN, PWM_MAX);
  controlos->pwm_motor_b = constrain(controlos->pwm_motor_b, PWM_MIN, PWM_MAX);
}

/**
 * **NOVO**: Imprime estado de navegação incluindo dados de heading
 */
void imprimir_estado_navegacao_vector() {
  // Fase actual
  Serial.print("FASE:");
  if (estado_nav.fase == FASE_APROXIMACAO) {
    Serial.print("APROX");
  } else {
    Serial.print("SEGUI");
  }
  
  // Posição GPS
  Serial.print(" | GPS:");
  Serial.print(estado_nav.latitude_actual, 6); 
  Serial.print(",");
  Serial.print(estado_nav.longitude_actual, 6);
  
  // Vector actual
  Serial.print(" | Vec:V");
  Serial.print(estado_nav.vector_actual.vertice_origem);
  Serial.print("→V");
  Serial.print(estado_nav.vector_actual.vertice_destino);
  
  // Progresso e distâncias
  Serial.print(" | Prog:");
  Serial.print(estado_nav.vector_actual.progresso_vector, 2);
  Serial.print(" | D.Dest:");
  Serial.print(estado_nav.vector_actual.distancia_ao_destino, 1);
  Serial.print("m | D.Vec:");
  Serial.print(estado_nav.vector_actual.distancia_ao_vector, 1);
  Serial.print("m");
  
  // Dados de heading
  Serial.print(" | Head:");
  Serial.print(estado_nav.brujula.heading_filtrado, 1);
  Serial.print("° | Bear:");
  Serial.print(estado_nav.bearing_objetivo, 1);
  Serial.print("° | Err:");
  Serial.print(controlos.erro_heading, 1);
  Serial.print("°");
  
  // PWM dos motores
  Serial.print(" | PWM A:");
  Serial.print(controlos.pwm_motor_a);
  Serial.print(" B:");
  Serial.println(controlos.pwm_motor_b);
}

void actualizar_gps() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }
}

// ========================================
//       GERAÇÃO DO POLÍGONO REGULAR
// ========================================

void gerar_poligono_regular(double baseLat, double baseLon, int num_vertices, double distancia_metros) {
  if (num_vertices > MAX_VERTICES) {
    Serial.print(">>> ERRO: Número de vértices excede limite máximo (");
    Serial.print(MAX_VERTICES);
    Serial.println(")");
    return;
  }
  
  total_vertices = num_vertices;
  
  double angulo_central = 2.0 * PI / num_vertices;
  double raio = distancia_metros / (2.0 * sin(PI / num_vertices));
  
  Serial.print(">>> Gerando polígono regular de ");
  Serial.print(num_vertices);
  Serial.print(" vértices com raio de ");
  Serial.print(raio, 2);
  Serial.println(" metros");
  
  for (int i = 0; i < num_vertices; i++) {
    double angulo = 90.0 - (i * 360.0 / num_vertices);
    vertices[i] = deslocar_coordenada_metros(baseLat, baseLon, raio, angulo);
  }
  
  Serial.print(">>> Polígono gerado com ");
  Serial.print(total_vertices);
  Serial.println(" vértices");
}

void imprimir_vertices_poligono() {
  Serial.println("\n>>> VÉRTICES DO POLÍGONO:");
  Serial.println("V# | Latitude  | Longitude | Dist.Próx");
  Serial.println("---|-----------|-----------|----------");
  
  for (int i = 0; i < total_vertices; i++) {
    int proximo = (i + 1) % total_vertices;
    double distancia = calcular_distancia_haversine(
      vertices[i].lat, vertices[i].lon,
      vertices[proximo].lat, vertices[proximo].lon
    );
    
    Serial.print("V");
    Serial.print(i);
    Serial.print(" | ");
    Serial.print(vertices[i].lat, 6);
    Serial.print(" | ");
    Serial.print(vertices[i].lon, 6);
    Serial.print(" | ");
    Serial.print(distancia, 1);
    Serial.println("m");
  }
}

bool verificar_vertice_atingido(const Ponto& posicao_barco, const Ponto& vertice_destino) {
  double distancia = calcular_distancia_haversine(
    posicao_barco.lat, posicao_barco.lon,
    vertice_destino.lat, vertice_destino.lon
  );
  return distancia <= RAIO_TOLERANCIA_VERTICE;
}

// ========================================
//          NAVEGAÇÃO AUTOMÁTICA
// ========================================

/**
 * **MODIFICADO**: Executa navegação por vectores com controlo de heading
 */
void executar_navegacao_por_vectores() {
  // Verificar fix GPS
  if (!gps.location.isValid()) {
    Serial.println(">>> AVISO: GPS sem fix válido!");
    parar_motores();
    return;
  }

  // Actualizar posição actual
  estado_nav.latitude_actual = gps.location.lat();
  estado_nav.longitude_actual = gps.location.lng();
  estado_nav.velocidade_kmh = gps.speed.kmph();

  // Verificar polígono
  if (total_vertices < 2) {
    Serial.println(">>> ERRO: Polígono não gerado ou insuficiente!");
    parar_motores();
    return;
  }

  Ponto posicao_barco = {estado_nav.latitude_actual, estado_nav.longitude_actual};

  // Actualizar estado do vector
  actualizar_estado_vector(&estado_nav.vector_actual, posicao_barco);

  // **NOVO**: Gestão de fases
  if (estado_nav.fase == FASE_APROXIMACAO) {
    // Verificar se chegou perto do vector
    if (estado_nav.vector_actual.distancia_ao_vector <= DISTANCIA_TRANSICAO_FASE) {
      Serial.println(">>> TRANSIÇÃO: APROXIMAÇÃO → SEGUIMENTO");
      estado_nav.fase = FASE_SEGUIMENTO;
      
      // Reset integral do PID ao mudar de fase
      controlos.pid_heading.integral = 0;
      controlos.pid_avanco.integral = 0;
    }
  }
  
  if (estado_nav.fase == FASE_SEGUIMENTO) {
    // Verificar se atingiu vértice destino
    if (verificar_vertice_atingido(posicao_barco, estado_nav.vector_actual.ponto_destino)) {
      Serial.print(">>> Vértice V");
      Serial.print(estado_nav.vector_actual.vertice_destino);
      Serial.println(" atingido! Mudando para próximo vector...");
      
      // Avançar para próximo vector
      estado_nav.vector_actual.vertice_origem = estado_nav.vector_actual.vertice_destino;
      estado_nav.vector_actual.vertice_destino = 
        (estado_nav.vector_actual.vertice_destino + 1) % total_vertices;
      
      estado_nav.vector_actual.ponto_origem = vertices[estado_nav.vector_actual.vertice_origem];
      estado_nav.vector_actual.ponto_destino = vertices[estado_nav.vector_actual.vertice_destino];
      
      // Voltar à fase de aproximação para o novo vector
      estado_nav.fase = FASE_APROXIMACAO;
      
      Serial.print("Novo vector: V");
      Serial.print(estado_nav.vector_actual.vertice_origem);
      Serial.print("→V");
      Serial.println(estado_nav.vector_actual.vertice_destino);
      
      // Reset PID
      controlos.pid_heading.integral = 0;
      controlos.pid_heading.erro_anterior = 0;
      controlos.pid_avanco.integral = 0;
      controlos.pid_avanco.erro_anterior = 0;
      
      // Recalcular estado
      actualizar_estado_vector(&estado_nav.vector_actual, posicao_barco);
    }
  }

  // Calcular controlos PID com heading
  calcular_controlos_pid_heading(&estado_nav, &controlos);
  
  // Aplicar controlos aos motores
  definir_velocidade_motores(controlos.pwm_motor_a, controlos.pwm_motor_b);

  // Mostrar estado
  imprimir_estado_navegacao_vector();
}

// ========================================
//      COMUNICAÇÃO COM ARDUINO NANO
// ========================================

void enviar_dados_nano() {
  if (modo_operacao != MODO_AUTOMATICO) {
    return;
  }

  if (!gps.location.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
    return;
  }

  // Timestamp
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

  // **MODIFICADO**: Linha CSV com dados de heading
  // Formato: Timestamp,Lat,Lon,VOrigem,VDestino,Fase,Progresso,DistDest,DistVec,Heading,Bearing,ErroHead,Vel,MotA,MotB
  String dados = fechaHora + "," +
                 String(estado_nav.latitude_actual, 6) + "," +
                 String(estado_nav.longitude_actual, 6) + "," +
                 String(estado_nav.vector_actual.vertice_origem) + "," +
                 String(estado_nav.vector_actual.vertice_destino) + "," +
                 String(estado_nav.fase) + "," +
                 String(estado_nav.vector_actual.progresso_vector, 3) + "," +
                 String(estado_nav.vector_actual.distancia_ao_destino, 2) + "," +
                 String(estado_nav.vector_actual.distancia_ao_vector, 2) + "," +
                 String(estado_nav.brujula.heading_filtrado, 2) + "," +
                 String(estado_nav.bearing_objetivo, 2) + "," +
                 String(controlos.erro_heading, 2) + "," +
                 String(estado_nav.velocidade_kmh, 2) + "," +
                 String(pwm_motor.motor_A) + "," +
                 String(pwm_motor.motor_B);

  Serial3.println(dados);
}

// ========================================
//             LOOP PRINCIPAL
// ========================================

void loop() {
  processar_comandos_rf24();
  actualizar_leds();
  actualizar_gps();
  
  // **NOVO**: Actualizar brújula
  actualizar_brujula();
  
  if (modo_operacao == MODO_AUTOMATICO && !erro_detectado) {
    executar_navegacao_por_vectores();
  }
  
  unsigned long tempo_actual = millis();
  if (sistema_iniciado && (tempo_actual - ultimo_envio_dados >= INTERVALO_ENVIO_DADOS_MS)) {
    ultimo_envio_dados = tempo_actual;
    enviar_dados_nano();
  }
  
  delay(150);
}

// ========================================
//           FUNÇÕES UTILITÁRIAS
// ========================================

double graus_para_radianos(double graus) {
  return graus * PI / 180.0;
}

double radianos_para_graus(double radianos) {
  return radianos * 180.0 / PI;
}

Ponto deslocar_coordenada_metros(double lat, double lon, double distancia_m, double angulo_graus) {
  const double RAIO_TERRA_M = 6371000.0;
  
  double angulo_rad = graus_para_radianos(90.0 - angulo_graus);
  
  double delta_x = distancia_m * cos(angulo_rad);
  double delta_y = distancia_m * sin(angulo_rad);
  
  double delta_lat = delta_y / RAIO_TERRA_M * (180.0 / PI);
  double delta_lon = delta_x / (RAIO_TERRA_M * cos(graus_para_radianos(lat))) * (180.0 / PI);
  
  Ponto novo_ponto;
  novo_ponto.lat = lat + delta_lat;
  novo_ponto.lon = lon + delta_lon;
  
  return novo_ponto;
}
