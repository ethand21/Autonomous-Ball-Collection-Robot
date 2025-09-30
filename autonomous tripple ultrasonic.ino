const int trigPin1 = 8;
const int echoPin1 = 9;
const int trigPin2 = 10;
const int echoPin2 = 11;
const int trigPin3 = 12;
const int echoPin3 = 13;
const int in1 = 3;
const int in2 = 4;
const int in3 = 5;
const int in4 = 6;



void setup() 
{
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);  
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);
  pinMode (in3, OUTPUT);
  pinMode (in4, OUTPUT);
}
long duration1, distance1;

long duration3, distance3;

long duration2, distance2;

void loop()
{     
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);  
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1/58.2;
  if(distance1<30)
    {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);  
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2/58.2;

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);  
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = duration3/58.2;

  if(distance3>distance2)
  {
      digitalWrite(in1, HIGH); 
      digitalWrite(in2, LOW); 
      digitalWrite(in3, LOW); 
      digitalWrite(in4, LOW);
  }
  else
  {
      digitalWrite(in1, LOW); 
      digitalWrite(in2, LOW); 
      digitalWrite(in3, HIGH); 
      digitalWrite(in4, LOW);
  }
    }
    
  else
    {
      digitalWrite(in1, HIGH); 
      digitalWrite(in2, LOW); 
      digitalWrite(in3, HIGH); 
      digitalWrite(in4, LOW);
    }  
  delay(0);
}

void right_hc()
{
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);  
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2/58.2;
}

void left_hc()
{
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);  
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = duration3/58.2;
}

