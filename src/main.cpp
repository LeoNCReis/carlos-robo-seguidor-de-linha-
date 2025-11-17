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

// CONSTANTES REVISADAS
const int SENSIBILIDADE = 750;
const int LONGPRESS = 2000;
const int BASE_SPEED = 200;           // Velocidade balanceada
const int AJUSTE_ESQUERDO = 25;
const int AJUSTE_DIREITO = 0;
const int TURN_SPEED = 180;           // Correção forte
const int WIGGLE_DURATION = 3000;
const float WAVE_FREQUENCY = 0.8;

// VARIÁVEIS GLOBAIS
unsigned long lastDebugTime = 0;
unsigned long debugInterval = 500;
bool ligado = false;
bool isWiggling = false;
bool linhaHorizontalDetectada = false;
unsigned long sineWaveStartTime = 0;
const unsigned long debounce = 50;
int turnFactor = 0;
int lastTurnFactor = 0;

// DECLARAÇÕES DE FUNÇÕES
void setMotorsBalanceados(int leftSpeed, int rightSpeed);
void parar();
void leiturasSensores();
void atualizarLeiturasDigitais();
void debugSensores();
bool verificarBotao();
void beep(int duration);
void seguirlinhaCorrigida();  // FUNÇÃO REVISADA
bool detectarLinhaHorizontal();

// INSTÂNCIAS
cIO robo;
AcksenButton button(BUT, ACKSEN_BUTTON_MODE_LONGPRESS, debounce, INPUT);

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

void debugSensores()
{
  if (millis() - lastDebugTime >= debugInterval) {
    lastDebugTime = millis();
    
    Serial.println("=== DEBUG SENSORES ===");
    Serial.print("LIGADO: "); Serial.println(ligado ? "SIM" : "NAO");
    Serial.print("WIGGLING: "); Serial.println(isWiggling ? "SIM" : "NAO");
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

// FUNÇÃO PRINCIPAL COMPLETAMENTE REVISADA
void seguirlinhaCorrigida()
{
  bool LE_ativo = robo.digitalReadings[cIO::LE_];
  bool CE_ativo = robo.digitalReadings[cIO::CE_];
  bool CD_ativo = robo.digitalReadings[cIO::CD_];
  bool LD_ativo = robo.digitalReadings[cIO::LD_];
  
  // DETECÇÃO MELHORADA PARA CURVAS DE 70-90 GRAUS
  // Padrão 1: Curva direita aguda (70-90º)
  if (LE_ativo && CE_ativo && !CD_ativo && !LD_ativo) {
    setMotorsBalanceados(BASE_SPEED - 200, BASE_SPEED + 80);
    Serial.println("CURVA DIREITA AGUDA");
    return;
  }
  
  // Padrão 2: Curva esquerda aguda (70-90º)  
  if (LD_ativo && CD_ativo && !CE_ativo && !LE_ativo) {
    setMotorsBalanceados(BASE_SPEED + 80, BASE_SPEED - 200);
    Serial.println("CURVA ESQUERDA AGUDA");
    return;
  }
  
  // Padrão 3: Curva direita muito fechada (apenas LE)
  if (LE_ativo && !CE_ativo && !CD_ativo && !LD_ativo) {
    setMotorsBalanceados(BASE_SPEED - 180, BASE_SPEED + 60);
    Serial.println("CURVA DIREITA MUITO FECHADA");
    return;
  }
  
  // Padrão 4: Curva esquerda muito fechada (apenas LD)
  if (LD_ativo && !CD_ativo && !CE_ativo && !LE_ativo) {
    setMotorsBalanceados(BASE_SPEED + 60, BASE_SPEED - 180);
    Serial.println("CURVA ESQUERDA MUITO FECHADA");
    return;
  }
  
  // Padrão 5: Linha deslocada à direita
  if (LE_ativo && CE_ativo && CD_ativo && !LD_ativo) {
    setMotorsBalanceados(BASE_SPEED - 120, BASE_SPEED + 50);
    return;
  }
  
  // Padrão 6: Linha deslocada à esquerda
  if (LD_ativo && CD_ativo && CE_ativo && !LE_ativo) {
    setMotorsBalanceados(BASE_SPEED + 50, BASE_SPEED - 120);
    return;
  }
  
  // Padrão 7: Curva suave direita
  if (LE_ativo && !CE_ativo && !CD_ativo && !LD_ativo) {
    setMotorsBalanceados(BASE_SPEED - 100, BASE_SPEED + 30);
    return;
  }
  
  // Padrão 8: Curva suave esquerda
  if (LD_ativo && !CD_ativo && !CE_ativo && !LE_ativo) {
    setMotorsBalanceados(BASE_SPEED + 30, BASE_SPEED - 100);
    return;
  }
  
  // LÓGICA PROPORCIONAL PARA CASOS NORMAIS
  turnFactor = 0;
  if (LE_ativo) turnFactor -= 3;  // Peso maior para sensores laterais
  if (CE_ativo) turnFactor -= 2;
  if (CD_ativo) turnFactor += 2;
  if (LD_ativo) turnFactor += 3;
  
  // Aplicar correção com histerese
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
  
  for (size_t i = 0; i < 4; i++) {
    pinMode(robo.bobinas[i], OUTPUT);
    pinMode(robo.sensores[i], INPUT);
  }
  
  pinMode(BUT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  
  digitalWrite(LED, LOW);
  digitalWrite(BUZZ, LOW);
  
  button.setLongPressInterval(LONGPRESS);
  
  Serial.println("=== SISTEMA INICIADO ===");
  Serial.println("MODO CORRIGIDO - CURVAS AGUDAS E PARADA CONFIÁVEL");
  Serial.print("BASE_SPEED: "); Serial.println(BASE_SPEED);
  Serial.print("TURN_SPEED: "); Serial.println(TURN_SPEED);
}

void loop()
{
  // 1. Ler sensores (SEMPRR primeiro)
  leiturasSensores();
  atualizarLeiturasDigitais();
  
  // 2. Verificar linha horizontal (ALTA PRIORIDADE)
  if (detectarLinhaHorizontal() && ligado && !linhaHorizontalDetectada) {
    Serial.println("*** LINHA HORIZONTAL DETECTADA - PARANDO ***");
    parar();
    linhaHorizontalDetectada = true;
    ligado = false;
    isWiggling = false;
    digitalWrite(LED, LOW);
    beep(500);
    delay(300);
    beep(500);
    return;
  }
  
  // 3. Debug
  debugSensores();
  
  // 4. Verificar botão
  if (verificarBotao()) {
    if (linhaHorizontalDetectada) {
      linhaHorizontalDetectada = false;
      ligado = true;
      Serial.println(">>> REINICIANDO APOS LINHA HORIZONTAL <<<");
      digitalWrite(LED, HIGH);
      beep(100); delay(100); beep(100);
    } else {
      ligado = !ligado;
      if (ligado) {
        Serial.println(">>> ROBO LIGADO - CORRIGIDO <<<");
        digitalWrite(LED, HIGH);
        beep(100); delay(100); beep(100);
      } else {
        Serial.println(">>> ROBO DESLIGADO <<<");
        digitalWrite(LED, LOW);
        parar();
        isWiggling = false;
        linhaHorizontalDetectada = false;
        beep(300);
      }
    }
  }

  // 5. Se linha horizontal foi detectada, ficar parado
  if (linhaHorizontalDetectada) {
    parar();
    digitalWrite(LED, LOW);
    return;
  }

  // 6. Lógica principal se estiver ligado
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