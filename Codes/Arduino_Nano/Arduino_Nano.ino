/**
 * ============================================================================
 *            SISTEMA DE REGISTO DE DADOS DE NAVEGAÇÃO EM CARTÃO SD
 * ============================================================================
 * 
 * OTIMIZADO PARA ARDUINO NANO (2KB RAM)
 * VERSÃO 2.1 - Formato de dados corrigido para incluir Bearing
 * 
 * Este programa recebe dados de navegação através da porta série e guarda-os
 * num cartão SD de forma organizada por data e hora.
 * 
 * BIBLIOTECA NECESSÁRIA:
 * - SdFat by Bill Greiman (instalar via Library Manager do Arduino IDE)
 * 
 * ORGANIZAÇÃO DOS FICHEIROS:
 * - Cria uma pasta com o nome da data (ex: "20251001" para 01/10/2025)
 * - Dentro da pasta, cria um ficheiro com o nome da hora (ex: "150955.txt")
 * - Cada linha recebida é guardada no ficheiro com um timestamp relativo
 * 
 * FORMATO DOS DADOS RECEBIDOS (CORRIGIDO):
 * 20251017_153045,38.691410,-9.299446,0,1,125.50,2.35,225.80,300.3,81.3,1500,1550
 * 
 * Campos:
 * - timestamp (data_hora)
 * - latitude
 * - longitude
 * - ponto atual
 * - ponto destino
 * - distância ao destino (m)
 * - erro perpendicular (m)
 * - heading (graus)
 * - bearing/angulo_vetor (graus) ← NOVO CAMPO
 * - erro angular/angulo_alinhado (graus)
 * - PWM motor A
 * - PWM motor B
 * 
 * Autor: Sistema de Navegação Autónoma
 * Data: Outubro 2025
 * ============================================================================
 */

#include <SPI.h>
#include "SdFat.h"

// ============================================================================
//                              CONFIGURAÇÕES
// ============================================================================
#define PIN_CS_SD 10
#define BAUD_RATE 9600
#define MAX_LINE 130          // Aumentado para acomodar campo extra

// ============================================================================
//                VARIÁVEIS GLOBAIS (uso mínimo de RAM)
// ============================================================================
SdFat sd;
SdFile ficheiro;
bool criado = false;
unsigned long t0 = 0;
char buffer[MAX_LINE];        // Buffer para linha recebida

// Strings em PROGMEM (guardadas na Flash, não na RAM)
const char msg1[] PROGMEM = "SD OK";
const char msg2[] PROGMEM = "Erro SD";
const char msg3[] PROGMEM = "Aguardando dados...";
const char msg4[] PROGMEM = "Pasta: ";
const char msg5[] PROGMEM = "Ficheiro: ";
const char msg6[] PROGMEM = "Erro criar pasta";
const char msg7[] PROGMEM = "Erro criar ficheiro";
const char cabecalho[] PROGMEM = "Tempo,Lat,Lon,PontoAtual,PontoDestino,DistDestino,ErroPerpend,Heading,Bearing,ErroAngular,PWM_A,PWM_B";

// ============================================================================
//                FUNÇÃO AUXILIAR: Imprimir string da Flash
// ============================================================================
void printP(const char* str) {
  char c;
  while ((c = pgm_read_byte(str++))) Serial.write(c);
}

// ============================================================================
//                                    SETUP
// ============================================================================
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);
  
  delay(1000);
  
  if (!sd.begin(PIN_CS_SD)) {
    printP(msg2);
    Serial.println();
    while(1);
  }
  
  printP(msg1);
  Serial.println();
  printP(msg3);
  Serial.println();
}

// ============================================================================
//                                     LOOP
// ============================================================================
void loop() {
  if (Serial.available() > 0) {
    
    // Ler linha para buffer
    byte idx = 0;
    unsigned long t_inicio = millis();
    
    while (millis() - t_inicio < 1000 && idx < MAX_LINE - 1) {
      if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          if (idx > 0) break;
          else continue;
        }
        buffer[idx++] = c;
      }
    }
    buffer[idx] = '\0';  // Terminar string
    
    // Validar tamanho mínimo
    if (idx < 16) {
      Serial.print(F("Linha curta: "));
      Serial.println(idx);
      return;
    }
    
    Serial.print(F("RX: "));
    Serial.println(buffer);
    
    // Extrair data e hora (usar ponteiros, não criar novas strings)
    // Data: buffer[0..7]
    // Hora: buffer[9..14]
    // Dados: buffer[16..]
    
    // ========================================================================
    //                CRIAR PASTA E FICHEIRO (primeira vez)
    // ========================================================================
    if (!criado) {
      
      // Criar nome da pasta (copiar data)
      char pasta[9];
      memcpy(pasta, buffer, 8);
      pasta[8] = '\0';
      
      // Verificar/criar pasta
      if (!sd.exists(pasta)) {
        printP(msg4);
        Serial.println(pasta);
        if (!sd.mkdir(pasta)) {
          printP(msg6);
          Serial.println();
          return;
        }
      }
      
      // Entrar na pasta
      if (!sd.chdir(pasta)) return;
      
      // Criar nome do ficheiro (copiar hora)
      char nome[13];
      memcpy(nome, buffer + 9, 6);
      strcpy(nome + 6, ".txt");
      
      printP(msg5);
      Serial.println(nome);
      
      // Abrir ficheiro
      if (!ficheiro.open(nome, O_WRITE | O_CREAT | O_TRUNC)) {
        printP(msg7);
        Serial.println();
        return;
      }
      
      criado = true;
      t0 = millis();
      
      // Escrever cabeçalho
      char cab[120];
      strcpy_P(cab, cabecalho);
      ficheiro.println(cab);
      ficheiro.sync();
      
      Serial.println(F("Ficheiro criado - OK"));
    }
    
    // ========================================================================
    //                              GRAVAR DADOS
    // ========================================================================
    if (criado && ficheiro.isOpen()) {
      
      // Calcular tempo decorrido
      unsigned long seg = (millis() - t0) / 1000;
      unsigned int hh = seg / 3600;
      unsigned int mm = (seg % 3600) / 60;
      unsigned int ss = seg % 60;
      
      // Escrever timestamp relativo (HH:MM:SS)
      if (hh < 10) ficheiro.write('0');
      ficheiro.print(hh);
      ficheiro.write(':');
      if (mm < 10) ficheiro.write('0');
      ficheiro.print(mm);
      ficheiro.write(':');
      if (ss < 10) ficheiro.write('0');
      ficheiro.print(ss);
      ficheiro.write(',');
      
      // Escrever linha completa recebida
      ficheiro.println(buffer);
      
      // Sincronizar para garantir escrita
      if (ficheiro.sync()) {
        Serial.print(F("["));
        if (hh < 10) Serial.print('0');
        Serial.print(hh);
        Serial.print(':');
        if (mm < 10) Serial.print('0');
        Serial.print(mm);
        Serial.print(':');
        if (ss < 10) Serial.print('0');
        Serial.print(ss);
        Serial.println(F("] Guardado"));
      } else {
        Serial.println(F("Erro sync"));
      }
    }
  }
}

/**
 * ============================================================================
 *                    ALTERAÇÕES NA VERSÃO 2.1:
 * ============================================================================
 * 
 * 1. Cabeçalho corrigido para incluir campo "Bearing" (angulo_vetor)
 * 2. Campo "Alinhado" renomeado para "ErroAngular" (angulo_alinhado)
 * 3. MAX_LINE aumentado para 130 bytes
 * 4. Buffer do cabeçalho aumentado para 120 bytes
 * 
 * ============================================================================
 *                         FORMATO DO FICHEIRO FINAL:
 * ============================================================================
 * 
 * Tempo,Timestamp,Lat,Lon,PontoAtual,PontoDestino,DistDestino,ErroPerpend,Heading,Bearing,ErroAngular,PWM_A,PWM_B
 * 00:00:00,20251017_153045,38.691410,-9.299446,0,1,125.50,2.35,225.80,300.3,81.3,1500,1550
 * 00:00:01,20251017_153046,38.691415,-9.299450,0,1,125.45,2.30,225.85,300.5,81.5,1505,1555
 * ...
 * 
 * Onde:
 * - Tempo: tempo decorrido desde o início da gravação (HH:MM:SS)
 * - Heading: direção atual do USV (do IMU/compasso)
 * - Bearing: direção desejada (ângulo do vetor para o próximo waypoint)
 * - ErroAngular: diferença entre Heading e Bearing
 * - Restantes campos: dados do sistema principal
 * 
 * ============================================================================
 */