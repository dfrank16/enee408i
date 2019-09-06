#define leftWheel 11
#define inLeftA 9
#define inLeftB 10

#define rightWheel 3
#define inRightA 4
#define inRightB 5

const int pingOutM=12;
const int pingInM=13;

const int pingOutL=8;
const int pingInL=6;

const int pingOutR=2;
const int pingInR=7;

int motorComp = 5;
int motorSpeed = 30;

long pingMDist, pingLDist, pingRDist;

void setup() {
  pinMode(leftWheel, OUTPUT);
  pinMode(rightWheel, OUTPUT);
  pinMode(inLeftA, OUTPUT);
  pinMode(inLeftB, OUTPUT);
  pinMode(inRightA, OUTPUT);
  pinMode(inRightB, OUTPUT);

  pinMode(pingOutM,OUTPUT);
  pinMode(pingInM,INPUT);
  pinMode(pingOutL,OUTPUT);
  pinMode(pingInL,INPUT);
  pinMode(pingOutR,OUTPUT);
  pinMode(pingInR,INPUT);
  
  Serial.begin(9600);
}

void loop() {
  pingDistance();

  if (pingMDist <= 8) {
    Serial.print("Close to middle\n");
    moveBack();
    delay(1000);
    turnAround();
  } else if (pingLDist <= 8) {
    Serial.print("Close to left\n");
    turnRight();
  } else if (pingRDist <= 8) {
    Serial.print("Close to right\n");
    turnLeft();
  } else {
    moveStraight();
  }

  delay(100);
}

void pingDistance(void) {
  long dur;
  
  /* M */
  digitalWrite(pingOutM,LOW);
  delayMicroseconds(2);
  digitalWrite(pingOutM,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingOutM,LOW);

  dur=pulseIn(pingInM,HIGH);
  pingMDist = microsecondsToInches(dur);

  /* L */
  digitalWrite(pingOutL,LOW);
  delayMicroseconds(2);
  digitalWrite(pingOutL,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingOutL,LOW);

  dur=pulseIn(pingInL,HIGH);
  pingLDist = microsecondsToInches(dur);
  Serial.println();
  Serial.print(pingLDist);
  Serial.println();

  /* R */
  digitalWrite(pingOutR,LOW);
  delayMicroseconds(2);
  digitalWrite(pingOutR,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingOutR,LOW);

  dur=pulseIn(pingInR,HIGH);
  pingRDist = microsecondsToInches(dur);
}

void moveStraight(void) {
  digitalWrite(inLeftA, HIGH);
  digitalWrite(inLeftB, LOW);
  digitalWrite(inRightA, LOW);
  digitalWrite(inRightB, HIGH);
  
  analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed + motorComp); // Send PWM signal to motor B
  
  Serial.print("Straight\n");
}

void moveBack(void) {
  digitalWrite(inLeftA, LOW);
  digitalWrite(inLeftB, HIGH);
  digitalWrite(inRightA, HIGH);
  digitalWrite(inRightB, LOW);
  
  analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed + motorComp); // Send PWM signal to motor B
  
  Serial.print("Backwards\n");
}

void turnAround(void) {
  digitalWrite(inLeftA, LOW);
  digitalWrite(inLeftB, HIGH);
  digitalWrite(inRightA, LOW);
  digitalWrite(inRightB, HIGH);
  
  analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed + motorComp); // Send PWM signal to motor B
  
  Serial.print("Turn Around\n");

  delay(1000);
}

void turnLeft(void) {
  digitalWrite(inLeftA, LOW);
  digitalWrite(inLeftB, HIGH);
  digitalWrite(inRightA, LOW);
  digitalWrite(inRightB, HIGH);
  
  analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed + motorComp); // Send PWM signal to motor B
  
  Serial.print("Turn left\n");

  delay(500);
}

void turnRight(void) {
  digitalWrite(inLeftA, HIGH);
  digitalWrite(inLeftB, LOW);
  digitalWrite(inRightA, HIGH);
  digitalWrite(inRightB, LOW);
  
  analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed + motorComp); // Send PWM signal to motor B
  
  Serial.print("Turn right\n");
  
  delay(500);
}


long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}
