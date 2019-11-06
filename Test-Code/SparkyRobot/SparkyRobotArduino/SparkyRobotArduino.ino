/*
  This code was created by Joubel Boco, Katrina Gramzinski, and Marquel Hendricks in Fall 2018 for use on Sparky Robot (UMD ENEE408I).
  
  This code controls the two motors on the chassis, takes input from the Jetson TX2, and takes input from the three ping sensors.
  
*/

// setting pin numers for all of the necessary Arduino pins
const int pingLeft = 2;
const int pingMiddle = 7;
const int pingRight = 13;
int incomingByte = 0;
const int enablePinLeft = 6;
const int aPinLeft = 5;
const int bPinLeft = 4;
const int pwmLeft = 3;
const int wheel_speed = 60;

const int enablePinRight = 8;
const int aPinRight = 9;
const int bPinRight = 10;
const int pwmRight = 11;
int output[3];
int buf = 0 ;

void setup() {
  // initialize serial communication with Jetson:
  Serial.begin(9600);
  pinMode(aPinLeft, OUTPUT);
  pinMode(bPinLeft, OUTPUT);
  pinMode(pwmLeft, OUTPUT);
  pinMode(enablePinLeft, OUTPUT);

  pinMode(aPinRight, OUTPUT);
  pinMode(bPinRight, OUTPUT);
  pinMode(pwmRight, OUTPUT);
  pinMode(enablePinRight, OUTPUT);
   
}

// This loop constantly checks the input from the Jetson
void loop() {
  incomingByte = Serial.read();
  if ( buf !=  incomingByte && incomingByte !=-1 ) {
    buf = incomingByte;
//  Serial.println(incomingByte);
    if (incomingByte==1)
      left(wheel_speed);
    else if (incomingByte==2)
      right(wheel_speed);
    else if (incomingByte==3)
      forward(wheel_speed);
    else if (incomingByte==4)
      backward(wheel_speed);
    else if (incomingByte==5)
      roam();
    else 
      halt();
   } 
}

// This is the only loop which takes input from the ping sensors. The application is "wandering" with obstacle avoidance.
void roam(){
  // Checks if the incoming byte from the Jetson is telling it to stop.
  while (incomingByte!=0) {
    checkPingSensors();
    // Checks if anything is closeby. output[0] is left ping, output[1] is middle ping, and output[2] is right ping.
    if (output[0] < 30 || output[1] < 30 || output[2] < 30) {
      // if anything is VERY close to left or right, immediately turn the other way.
      // Assumes we're not navigating down a very small hallway
      // Uses wheel_speed-20 because the default speed was scarily fast for wandering.
      if (output[0] < 10)
        right(wheel_speed-5);
      else if (output[2] < 10)
        left(wheel_speed-5);
      // otherwise, if there's a large distance ahead, move forward.
      else if (output[1] > 30) {
        forward(wheel_speed-5);
      }
      // otherwise, if there's more space to the left, turn left
      else if (output[0] > output[2]) {
        left(wheel_speed-5);
      }
      // otherwise, turn right.
      else {
        right(wheel_speed-5);
      }
    }
    // if nothing closeby, move forward
    else
      forward(wheel_speed-5);
    delay(1);
    // read the next incoming byte
    incomingByte = Serial.read();
    // update the byte if necessary. Doesn't resave if there's no change to save computation time
    if ( buf !=  incomingByte && incomingByte !=-1 ) {
      buf = incomingByte;
      if (incomingByte == 0)
        halt();
        break;
    }
  }
  return;
}

void checkPingSensors(){ 
  long durationMiddle, durationLeft, durationRight, inchesMiddle, inchesRight, inchesLeft, cmMiddle, cmLeft, cmRight;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingLeft, OUTPUT);
  digitalWrite(pingLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(pingLeft, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingLeft, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingLeft, INPUT);
  durationLeft = pulseIn(pingLeft, HIGH);

  // convert the time into a distance
  cmLeft = microsecondsToCentimeters(durationLeft);
  
  // save distance in cm to output
  output[0] = cmLeft;

  pinMode(pingMiddle, OUTPUT);
  digitalWrite(pingMiddle, LOW);
  delayMicroseconds(2);
  digitalWrite(pingMiddle, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingMiddle, LOW);

  pinMode(pingMiddle, INPUT);
  durationMiddle = pulseIn(pingMiddle, HIGH);

  cmMiddle = microsecondsToCentimeters(durationMiddle);
  
  // save distance in cm to output
  output[1] = cmMiddle;

  pinMode(pingRight, OUTPUT);
  digitalWrite(pingRight, LOW);
  delayMicroseconds(2);
  digitalWrite(pingRight, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingRight, LOW);

  pinMode(pingRight, INPUT);
  durationRight = pulseIn(pingRight, HIGH);

  cmRight = microsecondsToCentimeters(durationRight);
  
  // save distance in cm to output
  output[2] = cmRight;

  delay(100);
}



long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

void left(int pwmSpeed){
  digitalWrite(aPinRight, HIGH);
  digitalWrite(bPinRight, LOW);
  digitalWrite(enablePinRight, HIGH);
  analogWrite(pwmRight, pwmSpeed);

  digitalWrite(aPinLeft, HIGH);
  digitalWrite(bPinLeft, LOW);
  digitalWrite(enablePinLeft, HIGH);
  analogWrite(pwmLeft, pwmSpeed);
}

void halt(){
  digitalWrite(aPinRight, LOW);
  digitalWrite(bPinRight, LOW);
  digitalWrite(enablePinRight, LOW);
  

  digitalWrite(aPinLeft, LOW);
  digitalWrite(bPinLeft, LOW);
  digitalWrite(enablePinLeft, LOW);
  
}

void right(int pwmSpeed){
  digitalWrite(aPinRight, LOW);
  digitalWrite(bPinRight, HIGH);
  digitalWrite(enablePinRight, HIGH);
  analogWrite(pwmRight, pwmSpeed);

  digitalWrite(aPinLeft, LOW);
  digitalWrite(bPinLeft, HIGH);
  digitalWrite(enablePinLeft, HIGH);
  analogWrite(pwmLeft, pwmSpeed);
}

void backward(int pwmSpeed){
  digitalWrite(aPinRight, LOW);
  digitalWrite(bPinRight, HIGH);
  digitalWrite(enablePinRight, HIGH);
  analogWrite(pwmRight, pwmSpeed-10);

  digitalWrite(aPinLeft, HIGH);
  digitalWrite(bPinLeft, LOW);
  digitalWrite(enablePinLeft, HIGH);
  analogWrite(pwmLeft, pwmSpeed);
}

void forward(int pwmSpeed){
  // put your main code here, to run repeatedly:
  digitalWrite(aPinRight, HIGH);
  digitalWrite(bPinRight, LOW);
  digitalWrite(enablePinRight, HIGH);
  analogWrite(pwmRight, pwmSpeed-10);

  digitalWrite(aPinLeft, LOW);
  digitalWrite(bPinLeft, HIGH);
  digitalWrite(enablePinLeft, HIGH);
  analogWrite(pwmLeft, pwmSpeed);
  
  delayMicroseconds(10);
  }
  int serialReadInt() {
  int ret = 0;
  for (int i = 1; i >= 0; i--) {
    while (!Serial.available()) {
    }
    ret = Serial.read();
    Serial.println(ret);
   // ret |= b << (i * 8);
  }
  return ret;
}