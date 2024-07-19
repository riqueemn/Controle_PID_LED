#include <Arduino.h>
#include <HardwareSerial.h>



const int ledPin = 9;       // Pino PWM para controlar o LED
const int sensorPin = A0;   // Pino analógico para ler o sensor de luz
int setPoint = 500;         // Valor alvo de luminosidade (ajuste conforme necessário)
int sensorValue = 0;        // Valor lido do sensor de luz
int pwmValue = 0;           // Valor do PWM a ser ajustado


// Parâmetros PID
float Kp = 2.0;             // Proporcional
float Ki = 0.5;             // Integral
float Kd = 1.0;             // Derivativo

// Variáveis para controle PID
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  sensorValue = analogRead(sensorPin); // Lê o valor do sensor de luz
  
  // Calcula o erro entre o setPoint e o valor lido do sensor
  error = setPoint - sensorValue;
  
  // Calcula a integral (soma dos erros)
  integral += error;
  
  // Calcula a derivada (taxa de variação do erro)
  derivative = error - previousError;
  
  // Calcula o valor do PWM usando a fórmula PID
  pwmValue = Kp * error + Ki * integral + Kd * derivative;
  
  // Limita o valor do PWM entre 0 e 255
  if (pwmValue > 255) {
    pwmValue = 255;
  } else if (pwmValue < 0) {
    pwmValue = 0;
  }
  
  // Aplica o valor do PWM ao LED
  analogWrite(ledPin, pwmValue);
  
  // Atualiza o erro anterior
  previousError = error;
  
  // Imprime os valores para depuração
  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(" PWM Value: ");
  Serial.println(pwmValue);

  // Pequeno delay para estabilizar o sistema
  delay(100);
}
