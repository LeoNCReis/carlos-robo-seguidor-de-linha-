#define TDIR 10  // trás direita
#define FDIR 11  // frente direita
#define TESQ 9   // trás esquerda
#define FESQ 6   // frente esquerda
#define LD A2    // sensor lateral direito
#define CD A3    // sensor central direito
#define CE A4    // sensor central esquerdo
#define LE A5    // sensor lateral esquerdo
#define BUT 7    // botão
#define LED 12   // LED
#define BUZZ 8   // Buzzer

// Pinos para o decodificador BCD
#define BCD_A 3
#define BCD_B 5  
#define BCD_C 4
#define BCD_D 2

#include <Arduino.h>
#include <math.h>
#include "AcksenButton.h"

// ENUMERAÇÕES E STRUCTS
struct controlIO
{
  enum Bobinas
  {
    TDIR_,
    FDIR_,
    TESQ_,
    FESQ_
  };

  enum Sensores
  {
    LD_,
    CD_,
    CE_,
    LE_
  };
  
  int bobinas[4] = {TDIR, FDIR, TESQ, FESQ};
  int sensores[4] = {LD, CD, CE, LE};
  char debugsensores[4][3] = {"LD", "CD", "CE", "LE"};
  int readings[4];
  bool digitalReadings[4];
} typedef cIO;

// CONSTANTES
const int SENSIBILIDADE = 750;
const int BASE_SPEED = 200;
const int AJUSTE_ESQUERDO = 25;
const int AJUSTE_DIREITO = 0;
const int TURN_SPEED = 180;
const int WIGGLE_DURATION = 3000;
const float WAVE_FREQUENCY = 0.8;

// VARIÁVEIS GLOBAIS
unsigned long lastDebugTime = 0;
unsigned long debugInterval = 500;
bool ligado = false;
bool isWiggling = false;
unsigned long sineWaveStartTime = 0;
const unsigned long debounce = 50;
int turnFactor = 0;
int lastTurnFactor = 0;

// NOVAS VARIÁVEIS PARA CONTADOR
int contadorFitais = 0;
bool ultimoEstadoHorizontal = false;
unsigned long lastBuzzerTime = 0;
const unsigned long BUZZER_DURATION = 300;

// DECLARAÇÕES DE FUNÇÕES
void setMotorsBalanceados(int leftSpeed, int rightSpeed);
void parar();
void leiturasSensores();
void atualizarLeiturasDigitais();
void debugSensores();
bool verificarBotao();
void beep(int duration);
void seguirlinhaCorrigida();
bool detectarLinhaHorizontal();
void displayDigit(int digit);
void atualizarDisplay();

// INSTÂNCIAS
cIO robo;
AcksenButton button(BUT, ACKSEN_BUTTON_MODE_NORMAL, debounce, INPUT);

void setMotorsBalanceados(int leftSpeed, int rightSpeed)
{
  leftSpeed += AJUSTE_ESQUERDO;
  rightSpeed += AJUSTE_DIREITO;
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  if (leftSpeed >= 0) {
    analogWrite(FESQ, leftSpeed);
    analogWrite(TESQ, 0);
  } else {
    analogWrite(FESQ, 0);
    analogWrite(TESQ, abs(leftSpeed));
  }
  
  if (rightSpeed >= 0) {
    analogWrite(FDIR, rightSpeed);
    analogWrite(TDIR, 0);
  } else {
    analogWrite(FDIR, 0);
    analogWrite(TDIR, abs(rightSpeed));
  }
}

void parar()
{
  analogWrite(FESQ, 0);
  analogWrite(TESQ, 0);
  analogWrite(FDIR, 0);
  analogWrite(TDIR, 0);
}

void leiturasSensores()
{
  robo.readings[cIO::LD_] = analogRead(LD);
  robo.readings[cIO::CD_] = analogRead(CD);
  robo.readings[cIO::CE_] = analogRead(CE);
  robo.readings[cIO::LE_] = analogRead(LE);
}

void atualizarLeiturasDigitais()
{
  for (int i = 0; i < 4; i++) {
    robo.digitalReadings[i] = (robo.readings[i] < SENSIBILIDADE);
  }
}

bool detectarLinhaHorizontal()
{
  for (int i = 0; i < 4; i++) {
    if (!robo.digitalReadings[i]) {
      return false;
    }
  }
  return true;
}

// FUNÇÃO PARA ATUALIZAR O DISPLAY BCD
void displayDigit(int digit)
{
  switch (digit) {
    case 0:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 1:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 2:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 3:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 4:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 5:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 6:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 7:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 8:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, HIGH);
      break;
    case 9:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, HIGH);
      break;
  }
}

void atualizarDisplay()
{
  displayDigit(contadorFitais % 10); // Mostra apenas unidades (0-9)
}

void debugSensores()
{
  if (millis() - lastDebugTime >= debugInterval) {
    lastDebugTime = millis();
    
    Serial.println("=== DEBUG SENSORES ===");
    Serial.print("LIGADO: "); Serial.println(ligado ? "SIM" : "NAO");
    Serial.print("WIGGLING: "); Serial.println(isWiggling ? "SIM" : "NAO");
    Serial.print("CONTADOR FITAS: "); Serial.println(contadorFitais);
    Serial.print("LINHA HORIZONTAL: "); Serial.println(detectarLinhaHorizontal() ? "SIM" : "NAO");
    Serial.print("TURNFACTOR: "); Serial.println(turnFactor);
    
    for (size_t i = 0; i < 4; i++) {
      Serial.print(robo.debugsensores[i]);
      Serial.print(": "); Serial.print(robo.readings[i]);
      Serial.print(" -> "); Serial.print(robo.digitalReadings[i] ? "LINHA" : "FORA");
      Serial.print(" | ");
    }
    Serial.println();
    Serial.println("=====================");
  }
}

bool verificarBotao() 
{
  button.refreshStatus();
  return button.onPressed();
}

void beep(int duration) 
{
  digitalWrite(BUZZ, HIGH);
  delay(duration);
  digitalWrite(BUZZ, LOW);
}

// FUNÇÃO PRINCIPAL COM CONTROLE DE CURVAS
void seguirlinhaCorrigida()
{
  bool LE_ativo = robo.digitalReadings[cIO::LE_];
  bool CE_ativo = robo.digitalReadings[cIO::CE_];
  bool CD_ativo = robo.digitalReadings[cIO::CD_];
  bool LD_ativo = robo.digitalReadings[cIO::LD_];
  
  // DETECÇÃO PARA CURVAS DE 70-90 GRAUS
  if (LE_ativo && CE_ativo && !CD_ativo && !LD_ativo) {
    setMotorsBalanceados(BASE_SPEED - 200, BASE_SPEED + 80);
    Serial.println("CURVA DIREITA AGUDA");
    return;
  }
  
  if (LD_ativo && CD_ativo && !CE_ativo && !LE_ativo) {
    setMotorsBalanceados(BASE_SPEED + 80, BASE_SPEED - 200);
    Serial.println("CURVA ESQUERDA AGUDA");
    return;
  }
  
  if (LE_ativo && !CE_ativo && !CD_ativo && !LD_ativo) {
    setMotorsBalanceados(BASE_SPEED - 180, BASE_SPEED + 60);
    Serial.println("CURVA DIREITA MUITO FECHADA");
    return;
  }
  
  if (LD_ativo && !CD_ativo && !CE_ativo && !LE_ativo) {
    setMotorsBalanceados(BASE_SPEED + 60, BASE_SPEED - 180);
    Serial.println("CURVA ESQUERDA MUITO FECHADA");
    return;
  }
  
  if (LE_ativo && CE_ativo && CD_ativo && !LD_ativo) {
    setMotorsBalanceados(BASE_SPEED - 120, BASE_SPEED + 50);
    return;
  }
  
  if (LD_ativo && CD_ativo && CE_ativo && !LE_ativo) {
    setMotorsBalanceados(BASE_SPEED + 50, BASE_SPEED - 120);
    return;
  }
  
  // LÓGICA PROPORCIONAL PARA CASOS NORMAIS
  turnFactor = 0;
  if (LE_ativo) turnFactor -= 3;
  if (CE_ativo) turnFactor -= 2;
  if (CD_ativo) turnFactor += 2;
  if (LD_ativo) turnFactor += 3;
  
  if (abs(turnFactor - lastTurnFactor) > 4) {
    turnFactor = lastTurnFactor + ((turnFactor > lastTurnFactor) ? 2 : -2);
  }
  
  int leftMotorSpeed = BASE_SPEED + (turnFactor * TURN_SPEED / 2);
  int rightMotorSpeed = BASE_SPEED - (turnFactor * TURN_SPEED / 2);
  
  setMotorsBalanceados(leftMotorSpeed, rightMotorSpeed);
  lastTurnFactor = turnFactor;
}

void setup()
{
  Serial.begin(115200);
  
  // Inicialização dos pinos dos motores e sensores
  for (size_t i = 0; i < 4; i++) {
    pinMode(robo.bobinas[i], OUTPUT);
    pinMode(robo.sensores[i], INPUT);
  }
  
  // Inicialização dos pinos do BCD
  pinMode(BCD_A, OUTPUT);
  pinMode(BCD_B, OUTPUT);
  pinMode(BCD_C, OUTPUT);
  pinMode(BCD_D, OUTPUT);
  
  pinMode(BUT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  
  digitalWrite(LED, LOW);
  digitalWrite(BUZZ, LOW);
  
  // Inicializar display com 0
  displayDigit(0);
  
  Serial.println("=== SISTEMA INICIADO ===");
  Serial.println("MODO CONTADOR AUTOMÁTICO - FITAS HORIZONTAIS");
  Serial.print("BASE_SPEED: "); Serial.println(BASE_SPEED);
}

void loop()
{
  // 1. Ler sensores
  leiturasSensores();
  atualizarLeiturasDigitais();
  
  // 2. Verificar e contar fitas horizontais
  bool linhaHorizontalAtual = detectarLinhaHorizontal();
  
  // Detecta transição: não estava na linha horizontal e agora está
  if (linhaHorizontalAtual && !ultimoEstadoHorizontal && ligado && !isWiggling) {
    contadorFitais++;
    Serial.print("*** FITA HORIZONTAL DETECTADA - CONTADOR: ");
    Serial.println(contadorFitais);
    
    // Tocar buzzer por 300ms
    digitalWrite(BUZZ, HIGH);
    lastBuzzerTime = millis();
    
    // Atualizar display
    atualizarDisplay();
  }
  ultimoEstadoHorizontal = linhaHorizontalAtual;
  
  // Desligar buzzer após 300ms
  if (digitalRead(BUZZ) == HIGH && (millis() - lastBuzzerTime >= BUZZER_DURATION)) {
    digitalWrite(BUZZ, LOW);
  }
  
  // 3. Debug
  debugSensores();
  
  // 4. Verificar botão (apenas para ligar/desligar)
  if (verificarBotao()) {
    ligado = !ligado;
    if (ligado) {
      Serial.println(">>> ROBO LIGADO - CONTADOR ATIVO <<<");
      digitalWrite(LED, HIGH);
      beep(100); delay(100); beep(100);
    } else {
      Serial.println(">>> ROBO DESLIGADO <<<");
      digitalWrite(LED, LOW);
      parar();
      isWiggling = false;
      // Não zera o contador ao desligar
      beep(300);
    }
  }

  // 5. Lógica principal se estiver ligado
  if (ligado) {
    // Verificar se todos os sensores estão na linha (coluna)
    bool todosNaLinha = true;
    for (int i = 0; i < 4; i++) {
      if (!robo.digitalReadings[i]) {
        todosNaLinha = false;
        break;
      }
    }
    
    if (todosNaLinha) {
      Serial.println("* COLUNA DETECTADA - INICIANDO WIGGLE *");
      isWiggling = true;
      sineWaveStartTime = millis();
      beep(300);
    }
    
    // Modo Wiggling (busca)
    if (isWiggling) {
      bool algumSensorNaLinha = false;
      for (int i = 0; i < 4; i++) {
        if (robo.digitalReadings[i]) {
          algumSensorNaLinha = true;
          break;
        }
      }
      
      if (algumSensorNaLinha && !todosNaLinha) {
        Serial.println("Linha encontrada durante busca!");
        isWiggling = false;
        beep(150); delay(80); beep(150);
      }
      else if (millis() - sineWaveStartTime < WIGGLE_DURATION) {
        float time = (millis() - sineWaveStartTime) / 1000.0;
        float sineValue = sin(time * 2.0 * PI * WAVE_FREQUENCY);
        
        int leftMotorSpeed = BASE_SPEED + (sineValue * TURN_SPEED);
        int rightMotorSpeed = BASE_SPEED - (sineValue * TURN_SPEED);
        
        setMotorsBalanceados(leftMotorSpeed, rightMotorSpeed);
      }
      else {
        Serial.println("Tempo de busca esgotado - PARANDO");
        parar();
        ligado = false;
        isWiggling = false;
        digitalWrite(LED, LOW);
      }
    }
    // Modo Seguidor de Linha Normal
    else {
      seguirlinhaCorrigida();
    }
  } else {
    parar();
  }
  
  delay(25);
}