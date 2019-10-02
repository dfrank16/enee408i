#define leftWheel 11
#define inRightA 9
#define inRightB 10

#define rightWheel 3
#define inLeftA 4
#define inLeftB 5

const int pingEchoM=12;
const int pingTrigM=13;

const int pingEchoL=8;
const int pingTrigL=6;

const int pingEchoR=2;
const int pingTrigR=7;

int motorComp = 6;
int motorSpeed = 35;

long pingMDist, pingLDist, pingRDist;

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

void loop() {
  pingDistance();

  if (pingMDist <= 15) {
    Serial.print("Close to middle\n");
    moveBack();
    delay(1000);
    turnAround();
  } else if (pingLDist <= 15) {
    Serial.print("Close to left\n");
    turnLeft();
  } else if (pingRDist <= 15) {
    Serial.print("Close to right\n");
    turnRight();
  } else {
    moveStraight();
  }

  delay(100);
}

void pingDistance(void) {
  long dur;
  
  /* M */
  digitalWrite(pingTrigM,LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigM,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingTrigM,LOW);

  dur=pulseIn(pingEchoM,HIGH);
  pingMDist = microsecondsToInches(dur);
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
  Serial.print("R-dist: ");
  Serial.println(pingRDist);
}

void moveStraight(void) {
  digitalWrite(inLeftA, HIGH);
  digitalWrite(inLeftB, LOW);
  digitalWrite(inRightA, LOW);
  digitalWrite(inRightB, HIGH);
  
  analogWrite(leftWheel, motorSpeed + motorComp); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed); // Send PWM signal to motor B
  
  Serial.print("Straight\n");
}

void moveBack(void) {
  digitalWrite(inLeftA, LOW);
  digitalWrite(inLeftB, HIGH);
  digitalWrite(inRightA, HIGH);
  digitalWrite(inRightB, LOW);
  
  analogWrite(leftWheel, motorSpeed+ motorComp); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed ); // Send PWM signal to motor B
  
  Serial.print("Backwards\n");
}

void turnAround(void) {
  digitalWrite(inLeftA, LOW);
  digitalWrite(inLeftB, HIGH);
  digitalWrite(inRightA, LOW);
  digitalWrite(inRightB, HIGH);
  
  analogWrite(leftWheel, motorSpeed+ motorComp); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed ); // Send PWM signal to motor B
  
  Serial.print("Turn Around\n");

  delay(1000);
}

void turnLeft(void) {
  digitalWrite(inLeftA, LOW);
  digitalWrite(inLeftB, HIGH);
  digitalWrite(inRightA, LOW);
  digitalWrite(inRightB, HIGH);
  
  analogWrite(leftWheel, motorSpeed + motorComp); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed); // Send PWM signal to motor B
  
  Serial.print("Turn left\n");

  delay(500);
}

void turnRight(void) {
  digitalWrite(inLeftA, HIGH);
  digitalWrite(inLeftB, LOW);
  digitalWrite(inRightA, HIGH);
  digitalWrite(inRightB, LOW);
  
  analogWrite(leftWheel, motorSpeed + motorComp); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed); // Send PWM signal to motor B
  
  Serial.print("Turn right\n");
  
  delay(500);
}


long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}
