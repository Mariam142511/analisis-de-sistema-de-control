#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <DHT.h>

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE); // Inicializar la pantalla LCD

// Pines para el puente H de la bomba de agua
const int in1Bomba = 8;  // Control de dirección IN1 (bomba)
const int in2Bomba = 7;  // Control de dirección IN2 (bomba)
const int pwmBomba = 11;  // Control PWM para la velocidad de la bomba

// Pines para el puente H del ventilador
const int in3Ventilador = 6;  // Control de dirección IN3 (ventilador)
const int in4Ventilador = 9;  // Control de dirección IN4 (ventilador)
const int pwmVentilador = 3;  // Pin PWM para controlar la velocidad del ventilador

// Pines para el puente H del sistema de luces LED
const int in1LED = 2;  // Control de dirección IN1 (LED)
const int in2LED = 4;  // Control de dirección IN2 (LED)
const int pwmLED = 5;  // Pin PWM para controlar la intensidad del LED

// Pines para el sensor DHT y sensor de humedad de la tierra
const int dhtPin = A2;  // Pin donde está conectado el DHT11
const int soilMoisturePin = A0;  // Pin para el sensor de humedad de la tierra

#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);

float tempC;
int soilMoistureValue;  // Valor del sensor de humedad de la tierra

// Variables para el PID del ventilador
double setpointFan = 24.0;  // Setpoint de temperatura para el ventilador
double fanOutput = 0;    // Salida del PID para el control del ventilador
double KpFan = 5.0, KiFan = 0.2, KdFan = 0.1;  // Ganancias del PID del ventilador
double prevInputFan = 0;
double integralFan = 0;
unsigned long lastTimeFan = 0;

// Variables para el PID de las luces LED
double setpointLED = 20.0;  // Setpoint de temperatura para las luces LED
double ledOutput = 0;  // Salida del PID para el control de las luces LED
double KpLED = 6.0, KiLED = 0.3, KdLED = 0.1;  // Ganancias del PID de las luces
double prevInputLED = 0;
double integralLED = 0;
unsigned long lastTimeLED = 0;

void setup() {
  // Inicializar LCD y puerto serial
  lcd.begin(16, 2);
  lcd.backlight();
  Serial.begin(115200);

  // Inicializar el sensor DHT11
  dht.begin();

  // Configurar los pines del puente H para la bomba
  pinMode(in1Bomba, OUTPUT);
  pinMode(in2Bomba, OUTPUT);
  pinMode(pwmBomba, OUTPUT);

  // Configurar los pines del puente H para el ventilador
  pinMode(in3Ventilador, OUTPUT);
  pinMode(in4Ventilador, OUTPUT);
  pinMode(pwmVentilador, OUTPUT);

  // Configurar los pines del puente H para el LED
  pinMode(in1LED, OUTPUT);
  pinMode(in2LED, OUTPUT);
  pinMode(pwmLED, OUTPUT);

  // Inicialmente apagar la bomba, el ventilador y el LED
  digitalWrite(in1Bomba, LOW);
  digitalWrite(in2Bomba, LOW);
  analogWrite(pwmBomba, 0);

  digitalWrite(in3Ventilador, LOW);
  digitalWrite(in4Ventilador, LOW);
  analogWrite(pwmVentilador, 0);

  digitalWrite(in1LED, LOW);
  digitalWrite(in2LED, LOW);
  analogWrite(pwmLED, 0);

  // Inicializar el tiempo para el cálculo del PID
  lastTimeFan = millis();
  lastTimeLED = millis();
}

void loop() {
  unsigned long currentTime = millis();  // Obtener el tiempo actual

  // Leer el valor de la temperatura del sensor DHT11
  tempC = dht.readTemperature();
  if (isnan(tempC)) {
    Serial.println("Error leyendo el DHT11");
    return;
  }

  // Leer el valor del sensor de humedad de la tierra
  soilMoistureValue = analogRead(soilMoisturePin);
  int soilMoisturePercentage = map(soilMoistureValue, 0, 1023, 0, 100);

  // Mostrar la temperatura y la humedad en el monitor serial
  Serial.print("Temperatura: ");
  Serial.print(tempC);
  Serial.println("°C");
  
  Serial.print("Humedad del suelo: ");
  Serial.print(soilMoisturePercentage);
  Serial.println("%");

  // Mostrar los valores en el LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEMP: ");
  lcd.print(tempC);
  lcd.print(" C");
  
  lcd.setCursor(0, 1);
  lcd.print("HUM.SUELO: ");
  lcd.print(soilMoisturePercentage);
  lcd.print("%");

  // Controlador ON-OFF para la bomba de agua
  if (soilMoisturePercentage < 40) {
    controlarBomba(255);  // Encender la bomba al máximo
    Serial.println("Bomba encendida.");
  } else {
    controlarBomba(0);   // Apagar la bomba
    Serial.println("Bomba apagada.");
  }

  // Calcular la salida del PID del ventilador
  fanOutput = computePID(setpointFan, tempC, KpFan, KiFan, KdFan, &integralFan, &prevInputFan, &lastTimeFan);

  // Controlar el ventilador con PWM basado en la salida del PID
  controlarVentilador(fanOutput);

  // Calcular la salida del PID de las luces LED
  ledOutput = computePID(setpointLED, tempC, KpLED, KiLED, KdLED, &integralLED, &prevInputLED, &lastTimeLED);

  // Controlar las luces LED con PWM basado en la salida del PID
  controlarLED(tempC);

  delay(1000);  // Actualizar cada segundo
}

// Función para calcular la salida del PID
double computePID(double setpoint, double input, double Kp, double Ki, double Kd, double *integral, double *prevInput, unsigned long *lastTime) {
  unsigned long now = millis();
  double timeChange = (double)(now - *lastTime) / 1000.0;  // Convertir a segundos

  // Error actual
  double error = setpoint - input;

  // Proporcional
  double output = Kp * error;

  // Integral (con anti-windup)
  *integral += (error * timeChange);
  if (*integral > 100) *integral = 100;   // Limitar la acumulación de la integral
  if (*integral < -100) *integral = -100;
  output += Ki * (*integral);

  // Derivativo
  double derivative = (input - *prevInput) / timeChange;
  output -= Kd * derivative;

  // Actualizar variables para la siguiente iteración
  *prevInput = input;
  *lastTime = now;

  return output;
}

void controlarVentilador(double pwmValue) {
  // Asegurarse que el valor PWM esté dentro del rango de 0 a 255
  pwmValue = constrain(pwmValue, -255, 255);

  if (pwmValue >= 0) {  // Error positivo
    digitalWrite(in3Ventilador, LOW);  // Apagar el ventilador
    digitalWrite(in4Ventilador, LOW);
    analogWrite(pwmVentilador, 0);     // Asegurar que el PWM está en 0
    Serial.println("Ventilador apagado.");
  } else if (pwmValue < 0) {  // Error negativo
    digitalWrite(in3Ventilador, HIGH);  // Configurar la dirección del ventilador
    digitalWrite(in4Ventilador, LOW);   // Dirección única
    int pwmOutput = map((int)(-pwmValue), 0, 255, 0, 255);  // Mapear valor negativo a positivo para el control del ventilador
    analogWrite(pwmVentilador, pwmOutput);  // Controlar la velocidad con PWM

    if (pwmOutput <= 127) {
      Serial.print("Ventilador encendido lentamente con PWM: ");
    } else {
      Serial.print("Ventilador encendido rápido con PWM: ");
    }
    Serial.println(pwmOutput);
  }
}
// Función para controlar las luces LED basado en la temperatura
void controlarLED(double tempC) {
  // Si la temperatura es mayor o igual a 20°C, apagar las luces
  if (tempC >= 20) {
    digitalWrite(in1LED, LOW);
    digitalWrite(in2LED, LOW);
    analogWrite(pwmLED, 0);
    Serial.println("Luces LED apagadas.");
  } else {
    // Si la temperatura está por debajo de 20°C, ajustar el brillo
    // Cuanto más lejos esté de 20°C, más brillarán los LEDs
    int intensidad = map(tempC, 0, 20, 255, 50);  // Mapear el rango de temperaturas a brillo
    intensidad = constrain(intensidad, 50, 100);  // Limitar el brillo entre 50 y 255
    digitalWrite(in1LED, HIGH);
    digitalWrite(in2LED, LOW);
    analogWrite(pwmLED, intensidad);
    Serial.print("Luces LED encendidas con PWM: ");
    Serial.println(intensidad);
  }
}

// Función para controlar la bomba basado en el PWM
void controlarBomba(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  
  if (velocidad > 0) {
    digitalWrite(in1Bomba, HIGH);
    digitalWrite(in2Bomba, LOW);
    analogWrite(pwmBomba, velocidad);
    Serial.println("Bomba encendida.");
  } else {
    digitalWrite(in1Bomba, LOW);
    digitalWrite(in2Bomba, LOW);
    analogWrite(pwmBomba, 0);
    Serial.println("Bomba apagada.");
  }
}
