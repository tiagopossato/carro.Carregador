#include "PID_v1.h"
//#define DEBUG
//*******DEFINICAO DOS PINOS*****/////
const int pinoTensao = P1_4;
const int pinoCorrente = P1_0;
const int pinoControle = P2_1;  
const int LM35 = P1_3;

//*******PARAMETROS DO CARREGADOR********///
double correnteDeCarga = 4.0f;//Amperes 'C'
double tensaoDeCarga = 4.1f; //Volts
const float correnteMaxSaida = correnteDeCarga * 2.0f; //Amperes
const float tensaoBatMorta = 1.9f; //Volts
const float tensaoPreCarga = 2.8f; //Volts
const float tensaoMaximaSaida = 10.0f; //Volts, usado para calcular a tensao lida, o potenciometro fica ajustado para 10V!

//*******RESISTOR SHUNT ********///
const float resistorShunt = 0.251f; //Ohms
const float resistenciaCabos= 0.1f; //Ohms

//******PID******//
//Define Variables we'll be connecting to
double Input, Output, Setpoint;
double consKp=2, consKi=4, consKd=1;

//Specify the links and initial tuning parameters
PID pidControle(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

/**Funcao de regressao linear com com valores em ponto flutuante*/
float converte(float x, float in_min, float in_max, float out_min, float out_max);

float leTensao();
float leCorrente();

void preCarga(float tensao, float corrente);
void cargaCorrente(float tensao, float corrente);
void cargaTensao(float tensao, float corrente);

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  analogFrequency(15000);
  analogReference(INTERNAL2V5);//altera a refencia do analogico para 2,5 V internos, evitando problemas com variações de tensao na alimentação.

  //initialize the variables we're linked to
  Input = 0.0f;
  Setpoint = correnteDeCarga;
  //coloca o PID em automatico
  pidControle.SetMode(AUTOMATIC);
}

void loop() {

  static char estado = 0;
#ifdef DEBUG
  String msg = String("");
#endif
  float tensao = leTensao();
  float corrente = leCorrente();

  if(corrente > correnteMaxSaida){
    analogWrite(pinoControle, 0);
#ifdef DEBUG
    Serial.println("Sobrecorrente na carga!");
#endif
    exit(-1);
  }

  switch(estado){
  case 0://bateria nao detectada
    if(tensao <= tensaoBatMorta) estado = 0;
    if(tensao > tensaoBatMorta) estado = 1;
    delay(1000);
    break;

  case 1:
    if(tensao > tensaoDeCarga) estado = 2;
    if(tensao <= tensaoDeCarga) estado = 3;
    break;

  case 2://bateria nao reconhecida
#ifdef DEBUG
    msg = String("Bateria nao reconhecida");
#endif
    estado = 1;
    break;

  case 3://inicia carga
#ifdef DEBUG
    msg = String("Iniciando carga");
#endif
    if(tensao > tensaoPreCarga) estado = 5;
    if(tensao <= tensaoPreCarga) estado = 4;
    break;

  case 4://pre-carga com 0.1C
    if(tensao > tensaoPreCarga) estado = 5;
    if(tensao <= tensaoPreCarga) estado = 4;
    //Pre-carga
    preCarga(tensao, corrente);
    break;

  case 5:
#ifdef DEBUG
    msg = String("Pre-carga encerrada");
#endif
    if(tensao > tensaoDeCarga) estado = 7;
    if(tensao <= tensaoDeCarga) estado = 6;
    break;

  case 6://carga com 1.0C
    static long tempoCargaCorrenteAnterior = 0;
    if(millis() - tempoCargaCorrenteAnterior > 4000 ) {
      tempoCargaCorrenteAnterior = millis();
      if(tensao > tensaoDeCarga){
        estado = 7; 
        break;
      }
      if(tensao <= tensaoDeCarga) estado = 6;
    }
    cargaCorrente(tensao, corrente);
    break;

  case 7://carga com tensao constante 
    static long tempoCargaTensaoAnterior = 0;
    if(millis() - tempoCargaTensaoAnterior > 4000 ) {
      if(tensao>=tensaoDeCarga){
        tempoCargaTensaoAnterior = millis();
        if(corrente >= (correnteDeCarga / 10.0f)) estado = 7;
        if(corrente < (correnteDeCarga / 10.0f)){
          analogWrite(pinoControle, 0);
          estado = 8;
#ifdef DEBUG
          Serial.print(tensao);
          Serial.println(" V");
          Serial.print(corrente);
          Serial.println(" A");
          Serial.println("Carga concluida");
#endif
          break;
        }
      }
    }
    if(corrente >= correnteDeCarga){
      cargaCorrente(tensao, corrente);
      estado=6;
      break;
    }

    cargaTensao(tensao, corrente);
    break;

  case 8://carga concluida
    break;
  }

#ifdef DEBUG
  static long previousMillis = 0;
  if(millis() - previousMillis > 1000 ) {
    previousMillis = millis();
    Serial.print(tensao);
    Serial.println(" V");
    Serial.print(corrente);
    Serial.println(" A");
    Serial.println(msg);
  }
#endif
}

/**
Le a tensao na bateria.
Faz uma media de 20 leituras atenuar instabilidades
A tensao da bateria e igual a tensao total do circuito menos a tensao no resistor shunt
*/
float leTensao(){
  float tensao = 0.0f;
  for(int i=0;i<20;i++){
    tensao += (converte((float) analogRead(pinoTensao), 0.0, 1023.0, 0.0, tensaoMaximaSaida) - converte((float) analogRead(pinoCorrente), 0.0f, 1023.0f, 0.0f, 2.5f));
  }
  tensao  = tensao /20.0;

  return tensao;
}

float leCorrente(){
  float corrente = 0.0f;

  for(int i=0;i<100;i++){
    corrente += converte((float) analogRead(pinoCorrente), 0.0f, 1023.0f, 0.0f, 2.5f) / resistorShunt;
    delayMicroseconds(10);
  }
  return corrente/100.0;
}

float leTemperatura(){
  return (float(analogRead(LM35))*3.3f/(1023))/0.01;  
}

void preCarga(float tensao, float corrente){
  Input = corrente;
  Setpoint = correnteDeCarga * 0.1f;
  pidControle.Compute();
  analogWrite(pinoControle, Output); 
#ifdef DEBUG
  static long previousMillis = 0;
  if(millis() - previousMillis > 1000 ) {
    previousMillis = millis();
    Serial.println("pre-carga com 0.1 C");
  }
#endif
}

void cargaCorrente(float tensao, float corrente){
  Input = corrente;
  Setpoint = correnteDeCarga;
  pidControle.Compute();
  analogWrite(pinoControle, Output);
#ifdef DEBUG
  static long previousMillis = 0;
  if(millis() - previousMillis > 1000 ) {
    previousMillis = millis();
    Serial.print("Carga por corrente: ");
    Serial.println(Output);
  }
#endif
}

void cargaTensao(float tensao, float corrente){
  Input = tensao;
  Setpoint = tensaoDeCarga;
  pidControle.SetTunings(consKp, consKi, consKd);
  pidControle.Compute();
  analogWrite(pinoControle, Output);
#ifdef DEBUG
  static long previousMillis = 0;
  if(millis() - previousMillis > 1000 ) {
    previousMillis = millis();
    Serial.print("Carga por tensao: ");
    Serial.println(Output);
  }
#endif

}

/**Funcao de regressao linear com com valores em ponto flutuante*/
float converte(float x, float in_min, float in_max, float out_min,
float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

