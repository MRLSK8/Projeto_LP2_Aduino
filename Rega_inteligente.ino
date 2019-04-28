// inclui a biblioteca:
#include <LiquidCrystal_I2C.h>
#include <dht.h>

// define os pinos de conexão entre o Arduino e o Display LCD
const int rs = 12, en = 11;
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);

// variáveis do programa
const int pinoSensor = A0;
int limiarSeco; // Valor maximo de umidade do solo
const int tempoRega = 50; // Tempo de rega em segundos
int umidadeSolo = 0; // Recebe o valor da umidade do solo
const int pinoDHT11 = A3; // sensor de umidade e temperatura
dht DHT; // VARIÁVEL DO TIPO DHT

int pino_d = 2; //Pino ligado ao D0 do sensor
int pino_a = A2; //Pino ligado ao A0 do sensor
int val_d = 0; //Armazena o valor lido do pino digital
int val_a = 0; //Armazena o valor lido do pino analogico
const int RelePin = 7; // pino ao qual o Módulo Relé está conectado

// Medir fluxo de agua
#define LITER 403
 
volatile int pulse;
int liters = 0;

void setup() {
  pulse = 0;
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3),changeState,FALLING);
  // define o tamanho do Display LCD
  lcd.begin(16, 2);
  pinMode(RelePin, OUTPUT); // seta o pino como saída 
  digitalWrite(RelePin, LOW);  //desativa o pino
  // Define os pinos do sensor como entrada
  pinMode(pino_d, INPUT);
  pinMode(pino_a, INPUT);
}

void loop() {
  // Mede a umidade a cada segundo. Faz isso durante uma hora (3600 segundos).
  for(int i=0; i < 5; i++) {
    
    DHT.read11(pinoDHT11); 
    
    // Posiciona o cursor do LCD na coluna 0 linha 1
    // (Obs: linha 1 é a segunda linha, a contagem começa em 0
    lcd.setCursor(0, 0);
    // Exibe a mensagem no Display LCD:
    lcd.print("Umid Solo: ");
    // Faz a leitura do sensor de umidade do solo
    umidadeSolo = analogRead(pinoSensor);
    // Converte a variação do sensor de 0 a 1023 para 0 a 100
    umidadeSolo = map(umidadeSolo, 1023, 0, 0, 100);

    // Limite de umidade do solo
    limiarSeco = analogRead(A1);
    // Converte a variação do sensor de 0 a 1023 para 0 a 100
    limiarSeco = map(limiarSeco, 1023, 0, 0, 100);
    // Exibe a mensagem no Display LCD:
    lcd.print(umidadeSolo);
    lcd.print(" % ");

    lcd.setCursor(0, 1);
    lcd.print("Umid Max.: ");
    lcd.print(limiarSeco);
    lcd.print(" %  ");

    // Sensor de chuva
    //Le e arnazena o valor do pino digital
    val_d = digitalRead(pino_d);
    //Le e armazena o valor do pino analogico
    val_a = analogRead(pino_a);
    
    delay(500);
  }
  
  if( (val_a > 0 && val_a < 511) || (umidadeSolo >= limiarSeco))
  {
    // Posiciona o cursor do LCD na coluna 0 linha 1
    // (Obs: linha 1 é a segunda linha, a contagem começa em 0
    lcd.setCursor(0, 1);
    // Exibe a mensagem no Display LCD:
    lcd.print("Solo Encharcado ");
    // Espera o tempo estipulado
    delay(3000);
    liters = 0;
    digitalWrite(RelePin, LOW);  //desativa o pino
  }else if((val_a > 512 && val_a < 1024) || (umidadeSolo < limiarSeco))
  {
    // Posiciona o cursor do LCD na coluna 0 linha 1
    // (Obs: linha 1 é a segunda linha, a contagem começa em 0
    lcd.setCursor(0, 1);
    // Exibe a mensagem no Display LCD:
    digitalWrite(RelePin, HIGH); //aciona o pino
    lcd.print("Regando ");

    lcd.print(liters);
    lcd.println(" L     ");
    
    delay(3000);
  }
}

void changeState(){
    pulse += 1;
    if (pulse >LITER){
        pulse   = 0;
        liters += 1;
    }
}