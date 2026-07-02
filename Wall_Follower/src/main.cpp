#include <Arduino.h>

#define B_pin 1
//sensores
#define echoPin1 2
#define trigPin1 3
#define echoPin2 8
#define trigPin2 9
#define echoPin3 6
#define trigPin3 7
//driver
#define DIR1 16
#define PWM1 17
#define DIR2 19
#define PWM2 18

#define maxSpeed 220
#define minSpeed 80
#define maxdist1 18
#define maxdist2 25
#define basePWM 220


typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Input variables
int s1, s2, s3;
int sensor1();
int sensor2();
int sensor3();
boolean run_L, run_R;
int distance;
long duration; 
int pos = 0, ENCB, ENCA;

// Output variables
boolean ML, MR, RUN_L, RUN_R, ctrl;
void fwd(boolean RUN, int PWM_L, int PWM_R);
void turnRight(boolean RUN);
void turnLeft(boolean RUN);
void volta(boolean RUN);

//controller variables
int obj, e, eprev, ediff, eint, u, PWM_R, PWM_L;
float kp, ki, kd;

// Our finite state machines
fsm_t fsm1, fsm2, fsm3;

unsigned long interval, last_cycle;
unsigned long loop_micros;

// Set new state
void set_state(fsm_t& fsm, int new_state) 
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}


void readEncoder();
int read_distance(int trigPin, int echoPin);
void setMotor(int dir, int pwmVal, int pwm, int dirPin);


void setup() {
  Serial.begin(115200);

  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);




  s1 = false;
  s2 = false; 
  s3= false;
  run_R=true ;
  run_L = false;
  ctrl = false;
  interval = 35;


  e = 0;
  eint = 0;
  ediff = 0;
  eprev = 0;
  obj = 10;

  //constantes controlador PID
  kp = 4;
  ki = 0.005;
  kd = 8;

  set_state(fsm1, 0);
  set_state(fsm2, 0); 
  set_state(fsm3, 0); 
}

void loop() 
{
    // To measure the time between loop() calls
    //unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;
      
      // Read the inputs
      
      s2 = sensor2();
      s3 = sensor3();

      
      

      // FSM processing
      


      // Calculate next state for the second state machine
      if (fsm2.state == 0 && run_L) {
          fsm2.new_state = 2;
      } else if (fsm2.state == 1 && (s1 < maxdist1 && s2 > maxdist2)) {
          fsm2.new_state = 2;
      } else if (fsm2.state == 2 && (s1 < maxdist1 && s2 < maxdist2)) {
          fsm2.new_state = 3;
      } else if (fsm2.state == 2 && (s1 > maxdist1 && s2 > maxdist2)) {
          fsm2.new_state = 1;
      } else if (fsm2.state == 3 && (s1 < maxdist1 && s2 > maxdist2)) {
          fsm2.new_state = 2;

      }





      if (fsm3.state == 0 && run_R) {
        fsm3.new_state = 2;
      } else if (fsm3.state == 1 && (s3 < maxdist1 && s2 > maxdist2)) {
        fsm3.new_state = 2;
      } else if (fsm3.state == 2 && (s3 < maxdist1 && s2 < maxdist2)) {
        fsm3.new_state = 3;
      } else if (fsm3.state == 2 && (s3 > maxdist1 && s2 > maxdist2)) {
        fsm3.new_state = 1;
      } else if (fsm3.state == 3 && (s3 < maxdist1 && s2 > maxdist2)) {
        fsm3.new_state = 2;
      } else if (fsm3.state == 1 && (s3 < maxdist1 && s2 < maxdist2)) {
        fsm3.new_state = 3;
      } 


      // Update the states
      set_state(fsm1, fsm1.new_state);
      set_state(fsm2, fsm2.new_state);
      set_state(fsm3, fsm3.new_state);
      
      //Control algorithm:
      if(ctrl == 1){
        //Controlador PID
        if(run_L){ // se estiver a seguir a parede da esquerda 
          e = obj - s1;
        }

        if(run_R){ // se estiver a seguir a parede da direita 
          e = obj - s3;
        }

        eint = (eint + e);
        if(eint > 250) {
          eint = 250;
        }
        if(eint < -250){
          eint = -250;
        }

        ediff = (e - eprev);

        u = kp*e + ki*eint + kd*ediff;

        if(run_L){ // se estiver a seguir a parede da esquerda 
          PWM_R = basePWM - u;
          PWM_L = basePWM + u;
        }

        if(run_R){ // se estiver a seguir a parede da direita 
          PWM_R = basePWM + u;
          PWM_L = basePWM - u;
        }
        

        if(PWM_R > 255){
          PWM_R = 255;
        }
        if(PWM_R < 0){
          PWM_R = 0;
        }
         
        if(PWM_L > 255){
          PWM_L = 255;
        }
        if(PWM_L < 0){
          PWM_L = 0;
        }
      }

      

      // Actions set by the current state of the second state machine
      if (fsm2.state == 2){
        fwd(1, PWM_L, PWM_R);
        ctrl = 1;
      } else if (fsm2.state == 1){
        turnLeft(1);
      } else if (fsm2.state == 3){
        turnRight(1);
      }

      
      if (fsm3.state == 2){
        fwd(1, PWM_L, PWM_R);
        ctrl = 1;
      } else if (fsm3.state == 1){
        turnRight(1);
      } else if (fsm3.state == 3){
        turnLeft(1);
      }

      

      
    eprev = e;
    // Debug using the serial port
    
    Serial.print(" x1: ");
    Serial.print(fsm1.state);
    Serial.print(" RUNL: ");
    Serial.print(run_L);
    Serial.print(" x2: ");
    Serial.print(fsm2.state);
    Serial.print(" RUNR: ");
    Serial.print(run_R);
    Serial.print(" x3: ");
    Serial.print(fsm3.state);

    Serial.print("    S1: ");
    Serial.print(s1);
    Serial.print(" S2: ");
    Serial.print(s2);
    Serial.print(" S3: ");
    Serial.print(s3);
    
    Serial.print("    Erro: ");
    Serial.print(e);
    Serial.print(" ErroInt: ");
    Serial.print(eint);
    Serial.print(" ErroD: ");
    Serial.print(ediff);   
    Serial.print(" Input: ");
    Serial.print(u);

    Serial.print("    V direita: ");
    Serial.print(PWM_R);
    Serial.print(" V esquerda: ");
    Serial.print(PWM_L);
    Serial.println();
    }
}




void readEncoder(){
    int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}

int read_distance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 100000UL);
  distance = duration * 0.034 / 2; 

  return distance;
}

int sensor1(){
  int dist;
  return dist = read_distance(trigPin1, echoPin1);
}

int sensor2(){
  int dist;
  return dist = read_distance(trigPin2, echoPin2); 
}

int sensor3(){
  int dist;
  return dist = read_distance(trigPin3, echoPin3);
}

void turnLeft(boolean RUN){
  if(RUN){
    setMotor(1, maxSpeed, PWM2, DIR2);
    setMotor(1, minSpeed, PWM1, DIR1);
  } else{
    setMotor(1, 0, PWM2, DIR2);
    setMotor(1, 0, PWM1, DIR1);
  }
}

void turnRight(boolean RUN){
  if(RUN){
    setMotor(1, maxSpeed, PWM1, DIR1);
    setMotor(1, minSpeed, PWM2, DIR2);
  } else{
    setMotor(1, 0, PWM2, DIR2);
    setMotor(1, 0, PWM1, DIR1);
  }
}



void fwd(boolean RUN, int PWM_L, int PWM_R){
  if(RUN){
    setMotor(1, PWM_L, PWM1, DIR1);
    setMotor(1, PWM_R, PWM2, DIR2);
  } else{
    setMotor(1, 0, PWM2, DIR2);
    setMotor(1, 0, PWM1, DIR1);
  }
}

void volta(boolean RUN){
  if(RUN){
    setMotor(1, 250, PWM1, DIR1);
    setMotor(1, 50, PWM2, DIR2);
  } else{
    setMotor(1, 0, PWM2, DIR2);
    setMotor(1, 0, PWM1, DIR1);
  }
}

void setMotor(int dir, int pwmVal, int pwm, int dirPin){
  analogWrite(pwm, pwmVal);
  if( dir == 1 ) digitalWrite(dirPin, HIGH);
  else if( dir == -1 ) digitalWrite(dirPin, LOW);
}


