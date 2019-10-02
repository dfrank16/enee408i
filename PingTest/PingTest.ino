#define leftWheel 11
#define inLeftA 13
#define inLeftB 12

#define rightWheel 10
#define inRightA 8
#define inRightB 9

const int pingOutR=4;
const int pingInR=5;

const int pingOutL=1;
const int pingInL=2;

const int pingOutM=6;
const int pingInM=7;

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

  Serial.print("Middle distance is: ");
  Serial.println(pingMDist);

  Serial.print("Right distance is: ");
  Serial.println(pingRDist);

  Serial.print("Left distance is: ");
  Serial.println(pingLDist);
  

  delay(1000);
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

  delay(100);
  /* L */
  digitalWrite(pingOutL,LOW);
  delayMicroseconds(2);
  digitalWrite(pingOutL,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingOutL,LOW);

  dur=pulseIn(pingInL,HIGH);
  pingLDist = microsecondsToInches(dur);
  delay(100);

  /* R */
  digitalWrite(pingOutR,LOW);
  delayMicroseconds(2);
  digitalWrite(pingOutR,HIGH);
  delayMicroseconds(10);
  digitalWrite(pingOutR,LOW);

  dur=pulseIn(pingInR,HIGH);
  pingRDist = microsecondsToInches(dur);
  delay(100);
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
