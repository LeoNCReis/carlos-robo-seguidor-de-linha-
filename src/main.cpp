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

// Pinos para o decodificador BCD (display de 7 segmentos)
#define BCD_A 3
#define BCD_B 5  
#define BCD_C 4
#define BCD_D 2

#include <Arduino.h>
#include <math.h>
#include "AcksenButton.h"

// ENUMERAÇÕES E STRUCTS-------------------------------------------------------------------------------------------------------------------------------

// Struct para controle das I/Os do robô
struct controlIO
{
  enum Bobinas // enumerando para facilitar a identificação das bobinas
  {
    TDIR_,  // trás direita
    FDIR_,  // frente direita  
    TESQ_,  // trás esquerda
    FESQ_   // frente esquerda
  };

  enum Sensores // enumerando para facilitar a identificação dos sensores
  {
    LD_,    // sensor lateral direito
    CD_,    // sensor central direito
    CE_,    // sensor central esquerdo
    LE_     // sensor lateral esquerdo
  };
  
  int bobinas[4] = {TDIR, FDIR, TESQ, FESQ};       // array com os pinos das bobinas
  int sensores[4] = {LD, CD, CE, LE};              // array com os pinos dos sensores
  char debugsensores[4][3] = {"LD", "CD", "CE", "LE"}; // array com os nomes dos sensores para debug
  int readings[4];                                     // array para armazenar as leituras analógicas dos sensores
  bool digitalReadings[4];                             // array para armazenar as leituras digitais dos sensores
} typedef cIO;

// CONSTANTES GLOBAIS-------------------------------------------------------------------------------------------------------------------------------
const int SENSIBILIDADE = 700;    // valor limite para considerar sensor sobre a linha (quanto menor, mais sensível)
const int BASE_SPEED = 180;       // velocidade base do robô em movimento reto (reduzida para melhor controle)
const int AJUSTE_ESQUERDO = 14;   // ajuste de calibração para o motor esquerdo (compensa diferenças mecânicas)
const int AJUSTE_DIREITO = 0;     // ajuste de calibração para o motor direito
const int TURN_SPEED = 120;       // intensidade da correção nas curvas (reduzida para movimentos mais suaves)
const int WIGGLE_DURATION = 3000; // duração máxima da busca por linha perdida (3 segundos)
const float WAVE_FREQUENCY = 0.8; // frequência do movimento oscilatório durante a busca

// NOVAS CONSTANTES PARA CURVAS SUAVES
const int CURVE_SPEED = 150;        // velocidade reduzida para curvas normais
const int SHARP_TURN_SPEED = 100;   // velocidade ainda menor para curvas muito fechadas
const int MIN_MOTOR_SPEED = 80;     // velocidade mínima garantida para evitar parada total do motor

// BUZZER DESLIGADO POR PADRÃO - MUDE PARA true SE QUISER BARULHO
const bool BUZZER_ATIVO = false;    // controle global para ativar/desativar o buzzer

// VARIÁVEIS GLOBAIS-------------------------------------------------------------------------------------------------------------------------------
unsigned long lastDebugTime = 0;    // timestamp do último debug
unsigned long debugInterval = 500;  // intervalo entre debugs (ms)
bool ligado = false;                // estado do robô (ligado/desligado)
bool isWiggling = false;            // indica se o robô está no modo de busca por linha
unsigned long sineWaveStartTime = 0; // timestamp de início do movimento oscilatório
const unsigned long debounce = 50;  // tempo de debounce do botão
int turnFactor = 0;                 // fator de correção de direção atual
int lastTurnFactor = 0;             // fator de correção de direção anterior (para suavização)

// VARIÁVEIS PARA CONTADOR DE FITAS
int contadorFitais = 0;              // contador de fitas horizontais detectadas
bool ultimoEstadoHorizontal = false; // estado anterior da detecção de linha horizontal
unsigned long lastBuzzerTime = 0;    // timestamp do último acionamento do buzzer
const unsigned long BUZZER_DURATION = 300; // duração do beep do buzzer

// DECLARAÇÕES DE FUNÇÕES---------------------------------------------------------------------------------------------------------------------------
void setMotors(int leftSpeed, int rightSpeed);  // controla os motores com ajustes de calibração
void parar();                                   // para todos os motores
void leiturasSensores();                       // lê todos os sensores analógicos
void atualizarLeiturasDigitais();              // converte leituras analógicas para digitais
void debugSensores();                          // exibe informações de debug no serial
bool verificarBotao();                         // verifica se o botão foi pressionado
void beep(int duration);                       // emite um beep pelo buzzer (se ativo)
void seguirlinhaCorrigida();                   // algoritmo principal de seguimento de linha
bool detectarLinhaHorizontal();                // detecta se todos os sensores estão na linha
void displayDigit(int digit);                  // exibe um dígito no display de 7 segmentos
void atualizarDisplay();                       // atualiza o display com o contador atual

// INSTÂNCIAS---------------------------------------------------------------------------------------------------------------------------------------
cIO robo;                                       // instância da struct de controle de I/O
AcksenButton button(BUT, ACKSEN_BUTTON_MODE_NORMAL, debounce, INPUT); // instância do botão

// FUNÇÃO: setMotors - Controla os motores com ajustes de calibração e limites de segurança
void setMotors(int leftSpeed, int rightSpeed)
{
  // APLICAR AJUSTE DE BALANCEAMENTO APENAS PARA VELOCIDADES POSITIVAS
  // Isso evita aplicar correções quando os motores estão girando para trás
  int adjustedLeftSpeed = leftSpeed;
  int adjustedRightSpeed = rightSpeed;
  
  if (leftSpeed >= 0) {
    adjustedLeftSpeed = leftSpeed + AJUSTE_ESQUERDO;  // compensa motor mais fraco
  }
  if (rightSpeed >= 0) {
    adjustedRightSpeed = rightSpeed + AJUSTE_DIREITO; // compensa motor mais fraco
  }

  // Garantir que as velocidades estejam dentro dos limites do PWM (0-255)
  adjustedLeftSpeed = constrain(adjustedLeftSpeed, -255, 255);
  adjustedRightSpeed = constrain(adjustedRightSpeed, -255, 255);
  
  // Controle do motor esquerdo: define direção baseada no sinal da velocidade
  if (adjustedLeftSpeed >= 0) {
    // Movimento para frente: aciona pino FESQ, desliga TESQ
    analogWrite(FESQ, adjustedLeftSpeed);
    analogWrite(TESQ, 0);
  } else {
    // Movimento para trás: aciona pino TESQ, desliga FESQ  
    analogWrite(FESQ, 0);
    analogWrite(TESQ, abs(adjustedLeftSpeed));
  }
  
  // Controle do motor direito: define direção baseada no sinal da velocidade
  if (adjustedRightSpeed >= 0) {
    // Movimento para frente: aciona pino FDIR, desliga TDIR
    analogWrite(FDIR, adjustedRightSpeed);
    analogWrite(TDIR, 0);
  } else {
    // Movimento para trás: aciona pino TDIR, desliga FDIR
    analogWrite(FDIR, 0);
    analogWrite(TDIR, abs(adjustedRightSpeed));
  }
}

// FUNÇÃO: parar - Desliga todos os motores imediatamente
void parar()
{
  analogWrite(FESQ, 0);
  analogWrite(TESQ, 0);
  analogWrite(FDIR, 0);
  analogWrite(TDIR, 0);
}

// FUNÇÃO: leiturasSensores - Lê os valores analógicos de todos os sensores
void leiturasSensores()
{
  robo.readings[cIO::LD_] = analogRead(LD);
  robo.readings[cIO::CD_] = analogRead(CD);
  robo.readings[cIO::CE_] = analogRead(CE);
  robo.readings[cIO::LE_] = analogRead(LE);
}

// FUNÇÃO: atualizarLeiturasDigitais - Converte leituras analógicas para valores booleanos
// Sensores sobre a linha retornam valor analógico BAIXO, por isso a comparação com SENSIBILIDADE
void atualizarLeiturasDigitais()
{
  for (int i = 0; i < 4; i++) {
    robo.digitalReadings[i] = (robo.readings[i] < SENSIBILIDADE);
  }
}

// FUNÇÃO: detectarLinhaHorizontal - Verifica se TODOS os sensores detectam linha simultaneamente
// Isso geralmente indica uma fita horizontal ou cruzamento
bool detectarLinhaHorizontal()
{
  for (int i = 0; i < 4; i++) {
    if (!robo.digitalReadings[i]) {
      return false; // se algum sensor não está na linha, não é uma linha horizontal
    }
  }
  return true; // todos os sensores estão sobre a linha
}

// FUNÇÃO: displayDigit - Controla o display de 7 segmentos via decodificador BCD
// O decodificador BCD converte 4 bits em um dígito de 0-9 no display
void displayDigit(int digit)
{
  switch (digit) {
    case 0: // 0000
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 1: // 0001
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 2: // 0010
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 3: // 0011
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 4: // 0100
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 5: // 0101
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 6: // 0110
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 7: // 0111
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 8: // 1000
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, HIGH);
      break;
    case 9: // 1001
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, HIGH);
      break;
  }
}

// FUNÇÃO: atualizarDisplay - Atualiza o display com o dígito das unidades do contador
void atualizarDisplay()
{
  displayDigit(contadorFitais % 10); // mostra apenas as unidades (0-9)
}

// FUNÇÃO: debugSensores - Exibe informações detalhadas no monitor serial para debugging
void debugSensores()
{
  if (millis() - lastDebugTime >= debugInterval) {
    lastDebugTime = millis();
    
    Serial.println("=== DEBUG SENSORES ===");
    Serial.print("LIGADO: "); Serial.println(ligado ? "SIM" : "NAO");
    Serial.print("WIGGLING: "); Serial.println(isWiggling ? "SIM" : "NAO");
    Serial.print("CONTADOR FITAS: "); Serial.println(contadorFitais);
    Serial.print("BUZZER: "); Serial.println(BUZZER_ATIVO ? "LIGADO" : "DESLIGADO");
    Serial.print("LINHA HORIZONTAL: "); Serial.println(detectarLinhaHorizontal() ? "SIM" : "NAO");
    Serial.print("TURNFACTOR: "); Serial.println(turnFactor);
    
    // Exibe leituras de todos os sensores
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

// FUNÇÃO: verificarBotao - Verifica se o botão foi pressionado (com debounce)
bool verificarBotao() 
{
  button.refreshStatus(); // atualiza o estado interno do botão
  return button.onPressed(); // retorna true apenas no instante do pressionamento
}

// FUNÇÃO: beep - Controla o buzzer com duração especificada (só funciona se BUZZER_ATIVO = true)
void beep(int duration) 
{
  if (BUZZER_ATIVO) {
    digitalWrite(BUZZ, HIGH);
    delay(duration);
    digitalWrite(BUZZ, LOW);
  }
}

// FUNÇÃO: seguirlinhaCorrigida - Algoritmo principal de seguimento de linha com correções suavizadas
// Esta função implementa um controle proporcional melhorado com detecção de padrões específicos
void seguirlinhaCorrigida()
{
  bool LE_ativo = robo.digitalReadings[cIO::LE_];
  bool CE_ativo = robo.digitalReadings[cIO::CE_];
  bool CD_ativo = robo.digitalReadings[cIO::CD_];
  bool LD_ativo = robo.digitalReadings[cIO::LD_];
  
  // DETECÇÃO INTELIGENTE DE TIPO DE CURVA
  // Identifica padrões específicos dos sensores para determinar o tipo de curva
  bool curvaEsquerda = LE_ativo || (LE_ativo && CE_ativo);
  bool curvaDireita = LD_ativo || (LD_ativo && CD_ativo);
  bool curvaAguda = (LE_ativo && CE_ativo && !CD_ativo && !LD_ativo) || 
                    (LD_ativo && CD_ativo && !CE_ativo && !LE_ativo);
  
  // SISTEMA DE VELOCIDADE ADAPTATIVA
  // Reduz a velocidade conforme a curvatura aumenta para melhor controle
  int velocidadeBase = BASE_SPEED;
  if (curvaAguda) {
    velocidadeBase = SHARP_TURN_SPEED; // velocidade mínima para curvas muito fechadas
    Serial.println("*** CURVA AGUDA - VELOCIDADE REDUZIDA ***");
  } else if (curvaEsquerda || curvaDireita) {
    velocidadeBase = CURVE_SPEED; // velocidade reduzida para curvas normais
    Serial.println("*** CURVA DETECTADA - VELOCIDADE REDUZIDA ***");
  }
  
  // PADRÕES ESPECÍFICOS PARA CURVAS DIREITAS
  // Detecta combinações específicas de sensores para aplicar correções otimizadas
  if (LE_ativo && CE_ativo && !CD_ativo && !LD_ativo) {
    // Curva direita aguda: motor esquerdo muito lento, direito rápido
    int leftSpeed = constrain(SHARP_TURN_SPEED - 120, MIN_MOTOR_SPEED, 255);
    int rightSpeed = constrain(SHARP_TURN_SPEED + 60, MIN_MOTOR_SPEED, 255);
    setMotors(leftSpeed, rightSpeed);
    Serial.println("CURVA DIREITA AGUDA SUAVIZADA");
    return;
  }
  
  // PADRÕES ESPECÍFICOS PARA CURVAS ESQUERDAS
  if (LD_ativo && CD_ativo && !CE_ativo && !LE_ativo) {
    // Curva esquerda aguda: motor direito muito lento, esquerdo rápido
    int leftSpeed = constrain(SHARP_TURN_SPEED + 60, MIN_MOTOR_SPEED, 255);
    int rightSpeed = constrain(SHARP_TURN_SPEED - 120, MIN_MOTOR_SPEED, 255);
    setMotors(leftSpeed, rightSpeed);
    Serial.println("CURVA ESQUERDA AGUDA SUAVIZADA");
    return;
  }
  
  // CORREÇÕES PARA CURVAS MENOS AGRESSIVAS
  // Padrões intermediários com correções mais suaves
  if (LE_ativo && !CE_ativo && !CD_ativo && !LD_ativo) {
    setMotors(velocidadeBase - 100, velocidadeBase + 50);
    Serial.println("CURVA DIREITA SUAVE");
    return;
  }
  
  if (LD_ativo && !CD_ativo && !CE_ativo && !LE_ativo) {
    setMotors(velocidadeBase + 50, velocidadeBase - 100);
    Serial.println("CURVA ESQUERDA SUAVE");
    return;
  }
  
  // PADRÕES DE TRANSição ENTRE CURVAS
  if (LE_ativo && CE_ativo && CD_ativo && !LD_ativo) {
    setMotors(velocidadeBase - 80, velocidadeBase + 40);
    return;
  }
  
  if (LD_ativo && CD_ativo && CE_ativo && !LE_ativo) {
    setMotors(velocidadeBase + 40, velocidadeBase - 80);
    return;
  }
  
  // CONTROLE PROPORCIONAL PADRÃO (PARA AJUSTES SUTIS)
  // Sistema de pesos: sensores laterais têm mais influência que os centrais
  turnFactor = 0;
  if (LE_ativo) turnFactor -= 2; // sensor esquerdo extremo: forte correção para direita
  if (CE_ativo) turnFactor -= 1; // sensor esquerdo central: correção moderada
  if (CD_ativo) turnFactor += 1; // sensor direito central: correção moderada  
  if (LD_ativo) turnFactor += 2; // sensor direito extremo: forte correção para esquerda
  
  // SUAVIZAÇÃO DE TRANSIÇÕES
  // Limita a variação brusca entre ciclos para evitar oscilações
  if (abs(turnFactor - lastTurnFactor) > 2) {
    turnFactor = lastTurnFactor + ((turnFactor > lastTurnFactor) ? 1 : -1);
  }
  
  // SELEÇÃO DE VELOCIDADE BASE INTELIGENTE
  // Usa velocidade de curva se estiver fazendo uma correção significativa
  int baseSpeed = (abs(turnFactor) >= 2) ? CURVE_SPEED : velocidadeBase;
  
  // CÁLCULO DAS VELOCIDADES DOS MOTORES
  // Motor esquerdo: base + correção | Motor direito: base - correção
  int leftMotorSpeed = baseSpeed + (turnFactor * TURN_SPEED);
  int rightMotorSpeed = baseSpeed - (turnFactor * TURN_SPEED);
  
  // GARANTIA DE VELOCIDADE MÍNIMA
  // Evita que os motores parem completamente, mantendo controle
  leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, 255);
  
  setMotors(leftMotorSpeed, rightMotorSpeed);
  lastTurnFactor = turnFactor; // armazena para suavização no próximo ciclo
}

//--------------------------------------------------------------------SETUP--------------------------------------------------------//-------------------------------
void setup()
{
  Serial.begin(115200); // inicializa comunicação serial para debugging
  
  // Inicializa todos os pinos de I/O
  for (size_t i = 0; i < 4; i++) {
    pinMode(robo.bobinas[i], OUTPUT);  // configura pinos dos motores como saída
    pinMode(robo.sensores[i], INPUT);  // configura pinos dos sensores como entrada
  }
  
  // Configura pinos do display BCD
  pinMode(BCD_A, OUTPUT);
  pinMode(BCD_B, OUTPUT);
  pinMode(BCD_C, OUTPUT);
  pinMode(BCD_D, OUTPUT);
  
  // Configura pinos adicionais
  pinMode(BUT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  
  // Estado inicial dos componentes
  digitalWrite(LED, LOW);   // LED desligado
  digitalWrite(BUZZ, LOW);  // Buzzer desligado
  
  displayDigit(0); // Inicializa display com zero
  
  // Mensagem de inicialização
  Serial.println("=== SISTEMA INICIADO ===");
  Serial.println("*** COMPORTAMENTO DE CURVAS SUAVIZADO ***");
  Serial.print("VELOCIDADE BASE: "); Serial.println(BASE_SPEED);
  Serial.print("VELOCIDADE CURVAS: "); Serial.println(CURVE_SPEED);
  Serial.print("VELOCIDADE CURVAS AGUDAS: "); Serial.println(SHARP_TURN_SPEED);
  Serial.print("VELOCIDADE MÍNIMA: "); Serial.println(MIN_MOTOR_SPEED);
  Serial.println("BUZZER: " + String(BUZZER_ATIVO ? "ATIVO" : "DESATIVADO"));
}

//--------------------------------------------------------------------LOOP--------------------------------------------------------//-------------------------------
void loop()
{
  // 1. LEITURA E PROCESSAMENTO DOS SENSORES
  leiturasSensores();              // lê valores analógicos dos sensores
  atualizarLeiturasDigitais();     // converte para valores booleanos
  
  // 2. CONTAGEM DE FITAS HORIZONTAIS
  // Detecta quando o robô passa sobre uma fita horizontal (cruzamento)
  bool linhaHorizontalAtual = detectarLinhaHorizontal();
  
  // Lógica de detecção de borda: conta apenas na transição de não-detecção para detecção
  if (linhaHorizontalAtual && !ultimoEstadoHorizontal && ligado && !isWiggling) {
    contadorFitais++;
    Serial.print("*** FITA HORIZONTAL DETECTADA - CONTADOR: ");
    Serial.println(contadorFitais);
    
    // Feedback audível (se buzzer ativo)
    if (BUZZER_ATIVO) {
      digitalWrite(BUZZ, HIGH);
      lastBuzzerTime = millis();
    }
    
    atualizarDisplay(); // atualiza o display com novo valor
  }
  ultimoEstadoHorizontal = linhaHorizontalAtual; // armazena estado para detecção de borda
  
  // Desligar buzzer após tempo determinado (evita beep contínuo)
  if (BUZZER_ATIVO && digitalRead(BUZZ) == HIGH && (millis() - lastBuzzerTime >= BUZZER_DURATION)) {
    digitalWrite(BUZZ, LOW);
  }
  
  // 3. DEBUG NO MONITOR SERIAL
  debugSensores(); // exibe informações de debug em intervalos regulares
  
  // 4. CONTROLE DO BOTÃO LIGA/DESLIGA
  if (verificarBotao()) {
    ligado = !ligado; // alterna estado do robô
    if (ligado) {
      Serial.println(">>> ROBO LIGADO <<<");
      digitalWrite(LED, HIGH); // LED indica robô ligado
      beep(100); delay(100); beep(100); // beep duplo de confirmação
    } else {
      Serial.println(">>> ROBO DESLIGADO <<<");
      digitalWrite(LED, LOW); // LED apagado
      parar();                // para motores imediatamente
      isWiggling = false;     // reseta estado de busca
      beep(300);              // beep longo de desligamento
    }
  }

  // 5. LÓGICA PRINCIPAL DE CONTROLE (apenas se robô estiver ligado)
  if (ligado) {
    // Verifica se todos os sensores detectam linha (indicando possível perda de linha)
    bool todosNaLinha = true;
    for (int i = 0; i < 4; i++) {
      if (!robo.digitalReadings[i]) {
        todosNaLinha = false;
        break;
      }
    }
    
    // MODO "WIGGLE": busca por linha perdida
    if (todosNaLinha) {
      Serial.println("* COLUNA DETECTADA - INICIANDO WIGGLE *");
      isWiggling = true;
      sineWaveStartTime = millis(); // marca início da busca
      beep(300); // beep indicando início da busca
    }
    
    if (isWiggling) {
      // Durante a busca, verifica se encontrou a linha novamente
      bool algumSensorNaLinha = false;
      for (int i = 0; i < 4; i++) {
        if (robo.digitalReadings[i]) {
          algumSensorNaLinha = true;
          break;
        }
      }
      
      // Se encontrou a linha durante a busca, volta ao modo normal
      if (algumSensorNaLinha && !todosNaLinha) {
        Serial.println("Linha encontrada durante busca!");
        isWiggling = false;
        beep(150); delay(80); beep(150); // beep duplo de confirmação
      }
      // Se ainda está no tempo de busca, executa movimento oscilatório
      else if (millis() - sineWaveStartTime < WIGGLE_DURATION) {
        float time = (millis() - sineWaveStartTime) / 1000.0; // tempo em segundos
        float sineValue = sin(time * 2.0 * PI * WAVE_FREQUENCY); // valor senoidal
        
        // Movimento oscilatório: os motores giram em sentidos opostos alternadamente
        int leftMotorSpeed = 200 + (sineValue * 30);  // oscila entre 170 e 230
        int rightMotorSpeed = 200 - (sineValue * 30); // oscila entre 170 e 230
        
        setMotors(leftMotorSpeed, rightMotorSpeed);
      }
      // Tempo de busca esgotado - desliga o robô
      else {
        Serial.println("Tempo de busca esgotado - PARANDO");
        parar();
        ligado = false;
        isWiggling = false;
        digitalWrite(LED, LOW); // LED apagado indica desligamento
      }
    }
    else {
      // MODO NORMAL: seguimento de linha com correções
      seguirlinhaCorrigida();
    }
  } else {
    // Robô desligado - garante que os motores estão parados
    parar();
  }
  
  delay(25); // Pequeno delay para estabilidade (aproximadamente 40Hz)
}