#define echoPin 8
#define trigPin 7

void setup() {
Serial.begin (9600);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
}

void loop() {
int distance;

distance=watch();
Serial.print(“Distance = “);
Serial.print(distance);
Serial.println(” cm”);
delay(500);

}
int watch(){
long echo_distance;
digitalWrite(trigPin,LOW);
delayMicroseconds(5);
digitalWrite(trigPin,HIGH);
delayMicroseconds(15);
digitalWrite(trigPin,LOW);
echo_distance=pulseIn(echoPin,HIGH);
echo_distance=echo_distance*0.01657; //how far away is the object in cm
Serial.println((int)echo_distance);
return echo_distance;
}