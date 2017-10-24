// ====================================================== TODO ===========================================================================================================\
// =======================================================================================================================================================================
// ===================================================== SOLVE PROBLEM WITH AUTO AND MANUAL ==============================================================================






//=======================================================================================================================================

//-------------------------------------- Libraries used -------------------------------------
#include <NewPing.h>
#include <ArduinoJson.h>

//-------------------------------------- Pins used ------------------------------------------
// Motor pins
#define motor1pin1  3
#define motor1pin2  5
#define motor2pin1  6
#define motor2pin2  9
// Ultrasonic sensor pins
#define TRIGGER     2
#define ECHO        4
                    
//------------------------------------- Variables ------------------------------------------
// Ultrasonic sensor
int MAX_DISTANCE =  15;
// MOTORS PWM
int CurrentSpeed = 255;
// Variable to read the serial port
String data;
// Boolean to know wether the bot is in auto or manual mode
boolean mode = true;
// Sets the Ultrasonic sensor
NewPing sonar(TRIGGER, ECHO, MAX_DISTANCE);

long previousMillis = 0;

String speed1, speed2;

int motor1, motor2;

boolean isSerial;

unsigned long currentMillis;

int interval = 100;

void setup() {
  // Begin conection with Android
  Serial.begin(9600);
  // Timeout in order to not wait for more data
  Serial.setTimeout(20);
  // Set the motor pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
}

void loop() {
  
  if (Serial.available())
  {
    isSerial = true;
    speed1 = Serial.readStringUntil(',');
    Serial.read();
    speed2 = Serial.readStringUntil(',');

    motor1 = speed1.toInt();
    motor2 = speed2.toInt();
    previousMillis = currentMillis;
    Move(motor1, motor2);
  }
  else {
    isSerial = false;
  }

  currentMillis = millis();
  if (currentMillis - previousMillis > interval && isSerial == false){
    BotStop();
  }
  
    
}

void Move(int motor1, int motor2)
{
  if(motor1 < 0){
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
//    Serial.println("motor1 negative");

    analogWrite(motor1pin2, -motor1);
  }
  if (motor1 >= 0){
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
//    Serial.println("motor1 positive");

    analogWrite(motor1pin1, motor1);
  }

  if(motor2 < 0){
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
//    Serial.println("motor2 negative");
  
    analogWrite(motor2pin2, -motor2);
  }
  if(motor2 >= 0){
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
//    Serial.println("motor2 positive");

    analogWrite(motor2pin1, motor2);
  }
}


// Bot stops
void BotStop()
{
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}


//========================================================= THE END ======================================================================
