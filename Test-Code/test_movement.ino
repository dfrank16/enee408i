#define leftWheel 11
#define inLeftA 9
#define inLeftB 10

#define rightWheel 3
#define inRightA 4
#define inRightB 5

const int out=12;
const int in=13;

int motorComp = 5;
int motorSpeed = 50;

void setup() {
  pinMode(leftWheel, OUTPUT);
  pinMode(rightWheel, OUTPUT);
  pinMode(inLeftA, OUTPUT);
  pinMode(inLeftB, OUTPUT);
  pinMode(inRightA, OUTPUT);
  pinMode(inRightB, OUTPUT);

  pinMode(out,OUTPUT);
  pinMode(in,INPUT);
  
  Serial.begin(9600);
}

void loop() {
  /*digitalWrite(inLeftA, HIGH);
  digitalWrite(inLeftB, LOW);
  digitalWrite(inRightA, LOW);
  digitalWrite(inRightB, HIGH);
  
  analogWrite(leftWheel, motorSpeed); // Send PWM signal to motor A
  analogWrite(rightWheel, motorSpeed + motorComp); // Send PWM signal to motor B
*/

  long dur;
  long dis;

  digitalWrite(out,LOW);
  delayMicroseconds(2);

  digitalWrite(out,HIGH);
  delayMicroseconds(10);
  digitalWrite(out,LOW);

  dur=pulseIn(in,HIGH);

  Serial.println(String(dur));

  delay(100);
}
