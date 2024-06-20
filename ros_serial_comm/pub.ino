
const int trigPin = 9;
const int echoPin = 10;

int distanceCm;
long duration;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  while(!Serial){}

}

void loop() {

//  Send
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * 0.034 / 2;
  Serial.println(distanceCm);
  delay(100);
 

//Receive 
//if (Serial.available() > 0){
//  String message = Serial.readStringUntil('\n');
//  message = message + " " + String(counter);
//  counter++;
//  Serial.println(message);
//}

}