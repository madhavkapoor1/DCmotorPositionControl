#include <util/atomic.h> // For ATOMIC_BLOCK
#include <TimerOne.h>

#define outputA 2
#define outputB 3
#define IN1 5
#define IN2 6
#define MESP 11

const int N = 12; // Filter length for WSF
float weights[N] = {0.3088, 0.2147, 0.1492, 0.1037, 0.0721, 0.0501, 0.0348, 0.0242, 0.0168, 0.0117, 0.0081, 0.0057};

volatile int counter = 0;
volatile int newPos = 0; 
volatile int target = 100; // Example target position
volatile float pwr = 0; // Motor power (speed)
volatile int dir = 1; // Motor direction

// PID constants
const float kp = 1.130663634375;
const float kd = 0.0310233579772417;
const float ki = 0.01592181425965;
volatile float eintegral = 0;
int maxIntegral = 5;
volatile float eprev = 0; // Previous error
long prevT = 0; // Previous time

void setup() {
  Timer1.initialize(400);
  Timer1.attachInterrupt(controlMotor); // Control motor in ISR

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  
  Serial.begin(9600);
  
  prevT = micros();
  
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  digitalWrite(outputA, 0);
  digitalWrite(outputB, 0);

  //testing
  // pinMode(MEASURE_PIN, OUTPUT);
  // pinMode(MEASURE_PIN_2, OUTPUT);

  delay(2000);
  
  counter = 0; 
  newPos = 0;
}

void loop() {
  // In the main loop, just apply the calculated motor power and direction
  setMotor(dir, (int)pwr, IN1, IN2);
  Serial.print(target);
  Serial.print(" ");
  Serial.print(counter);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int in1, int in2) {
  if(dir == 1){
    analogWrite(in1, pwmVal);
    analogWrite(in2, 0);
  } else if(dir == -1){
    analogWrite(in1, 0);
    analogWrite(in2, pwmVal);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void controlMotor() {
  analogWrite(MESP,255);

  static float dedtHistory[N] = {0}; // Derivative history
  
  int aState = digitalRead(outputA);
  static int aLastState = aState;
  newPos = counter;
  
  if (aState != aLastState) {
    if (digitalRead(outputB) != aState) {
      newPos++;
    } else {
      newPos--;
    }
  }
  aLastState = aState;
  counter = newPos;
  
  long currentT = micros();
  float deltaT = ((float)(currentT - prevT) / 1.0e6); //replace this with 1/CF // Time difference in seconds
  //change this to (current error - prev error)*CF
  prevT = currentT;

  int pos = newPos; // Use newPos to avoid multiple volatile accesses
  int e = target - pos;
  float dedt = (e - eprev) / deltaT;

  for (int i = N - 1; i > 0; i--) {
    dedtHistory[i] = dedtHistory[i - 1];
  }
  dedtHistory[0] = dedt;

  float filtdedt = 0;
  for (int i = 0; i < N; i++) {
    filtdedt += dedtHistory[i] * weights[i];
  }

  eintegral += e * deltaT;
  //if (eintegral > maxIntegral) eintegral = maxIntegral;
  //else if (eintegral < -maxIntegral) eintegral = -maxIntegral;

  float u = kp * e + kd * filtdedt + ki * eintegral;

  pwr = fabs(u);
  if (pwr > 255) pwr = 255;
  
  dir = (u < 0) ? -1 : 1;
  if (u = 0) pwr = 0;

  eprev = e;

  analogWrite(MESP,0);
}
