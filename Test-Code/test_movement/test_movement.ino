
#define leftWheel 10
#define inLeftA 8
#define inLeftB 9
//I think that the wheels are backwards
#define rightWheel 11
#define inRightA 13
#define inRightB 12

const int pingEchoM=7;
const int pingTrigM=6;

const int pingEchoL=2;
const int pingTrigL=1;

const int pingEchoR=5;
const int pingTrigR=4;

int motorComp = 6;
int motorSpeed = 40;

long pingMDist, pingLDist, pingRDist;

//int countL = 0;
//int countM = 0;
//int countR = 0;
//int countS = 0;

int state = 0;
// 0 -> stopped
// 1 -> forward
// 2 -> backward
// 3 -> left
// 4 -> right

int mode = 0;
// 1 -> move on ping sensors
// 0 -> move on camera tracking


void setup() {
  pinMode(leftWheel, OUTPUT);
  pinMode(rightWheel, OUTPUT);
  pinMode(inLeftA, OUTPUT);
  pinMode(inLeftB, OUTPUT);
  pinMode(inRightA, OUTPUT);
  pinMode(inRightB, OUTPUT);

  pinMode(pingEchoM,INPUT);
  pinMode(pingTrigM,OUTPUT);
  pinMode(pingEchoL,INPUT);
  pinMode(pingTrigL,OUTPUT);
  pinMode(pingEchoR,INPUT);
  pinMode(pingTrigR,OUTPUT);
  
  Serial.begin(9600);
}
int incomingByte = 0;
void loop() {

  if (mode) {
    pingModeMove();
  } else {
    camModeMove();
  }

}


void camModeMove(void) {
  if(Serial.available() > 0) {
    incomingByte = Serial.read();

    if (incomingByte == 1) {
      turnLeft();
    } else if(incomingByte == 3) {
      turnRight();
    } else if(incomingByte == 2) {
      moveStraight();
    } else if(incomingByte == 4) {
      moveStop();
    }
   incomingByte = 0; 
  }
}


void pingModeMove(void){
  pingDistance();
  if (pingMDist <= 20  && pingMDist != 0) {     
         moveBack();
         //Serial.print("Close to middle\n");
        delay(1000);
        turnAround();
  } else {
    if (pingLDist <= 15 && pingLDist != 0) {
         turnRight();  
    }
    else if(pingRDist <= 20  && pingRDist !=0) {
        turnLeft();
    } else {
    moveStraight();
    }
  }
}


void pingDistance(void) {
  long dur;
    delay(100);

  /* M */
  digitalWrite(pingTrigM,LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigM,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingTrigM,LOW);

  dur=pulseIn(pingEchoM,HIGH);
  pingMDist = microsecondsToInches(dur);
  //delay(100);
  Serial.print("M-dist: ");
  Serial.println(pingMDist);

  /* L */
  delay(100);
  digitalWrite(pingTrigL,LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigL,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingTrigL,LOW);

  dur=pulseIn(pingEchoL,HIGH);
  pingLDist = microsecondsToInches(dur);
  //delay(100);
  Serial.print("L-dist: ");
  Serial.println(pingLDist);


  /* R */
  delay(100);
  digitalWrite(pingTrigR,LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigR,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingTrigR,LOW);

  dur=pulseIn(pingEchoR,HIGH);
  pingRDist = microsecondsToInches(dur);
  //delay(100);
  Serial.print("R-dist: ");
  Serial.println(pingRDist);
}

void moveStop(void) {
  if(state != 0){
    digitalWrite(inLeftA, LOW);
    digitalWrite(inLeftB, LOW);
    digitalWrite(inRightA, LOW);
    digitalWrite(inRightB, LOW);
    state = 0;
    Serial.print("State is set to stop\n");
  }
  //Serial.print("Sate is Stop\n");
  
}

void moveStraight(void) {
  if(state != 1){
    digitalWrite(inLeftA, HIGH);
    digitalWrite(inLeftB, LOW);
    digitalWrite(inRightA, LOW);
    digitalWrite(inRightB, HIGH);
    analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
    analogWrite(rightWheel, motorSpeed); // Send PWM signal to motor B
    state = 1;
    Serial.print("State is set to Straight\n");
  }

  Serial.print("State is Straight\n");
}

void moveBack(void) {
  if(state != 2) {
    digitalWrite(inLeftA, LOW);
    digitalWrite(inLeftB, HIGH);
    digitalWrite(inRightA, HIGH);
    digitalWrite(inRightB, LOW);
  
    analogWrite(leftWheel, motorSpeed+ motorComp); // Send PWM signal to motor A
    analogWrite(rightWheel, motorSpeed ); // Send PWM signal to motor B
    state = 2;
    Serial.print("State is set to Backwards");
  } else{
   // Serial.print("State is Backwards\n");
  }
}

void turnAround(void) {
  
  turnLeft();
  delay(2000);
  moveStraight();
  
//  digitalWrite(inLeftA, LOW);
//  digitalWrite(inLeftB, HIGH);
//  digitalWrite(inRightA, LOW);
//  digitalWrite(inRightB, HIGH);
//  
//  analogWrite(leftWheel, motorSpeed+ motorComp); // Send PWM signal to motor A
//  analogWrite(rightWheel, motorSpeed ); // Send PWM signal to motor B
//  
  Serial.print("Turn Around\n");

 // delay(1000);
}

void turnLeft(void) {
  if(state != 3) {
    digitalWrite(inLeftA, HIGH);
    digitalWrite(inLeftB, LOW);
    digitalWrite(inRightA, HIGH);
    digitalWrite(inRightB, LOW);
    
    analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
    analogWrite(rightWheel, motorSpeed + motorComp); // Send PWM signal to motor B
    state = 3;
    Serial.print("State is set to Left\n");
    
  } else{ 
    Serial.print("State is Left\n");

  }

}

void turnRight(void) {
  if(state != 4){
    digitalWrite(inLeftA, LOW);
    digitalWrite(inLeftB, HIGH);
    digitalWrite(inRightA, LOW);
    digitalWrite(inRightB, HIGH);
    analogWrite(leftWheel, motorSpeed + motorComp); // Send PWM signal to motor A
    analogWrite(rightWheel, motorSpeed); // Send PWM signal to motor B
    state = 4;
    Serial.print("State is set to Right\n");
  } else {
    Serial.print("State is right\n"); 
  }
  
  
}


long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}
