/*double count = 0; //set the counts of the encoder
#double angle = 0;//set the angles
#boolean A,B;
#byte state, statep;*/
// this is the PWM pin for the motor for how much we move it to correct for its error

int dir = 2 ;//these pins are to control the direction of the motor (clockwise/counter-clockwise)


//double setpoint = 100;//I am setting it to move through 100 degrees
double Kp = 1 ;// you can set these constants however you like depending on trial & error
double Ki = 0;
double Kd = 0;

float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTerm_scaled = 0;

int setpoint = 70  ;
int angle ;

// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|

int pwm = 3;



void setup() {
  Serial.begin(9600);
  //pinMode(2, INPUT);//encoder pins
  pinMode(pwm, OUTPUT);
  /*attachInterrupt(0,Achange,CHANGE);//interrupt pins for encoder
  attachInterrupt(1,Bchange,CHANGE); 
  */
  pinMode(A0,INPUT);
  //pinMode(dir1, OUTPUT);
  pinMode(dir, OUTPUT);

}

void loop(){
  /*Serial.println(" angle ");
  setpoint = Serial.parseInt(); 
  while(setpoint==0){
    setpoint = Serial.parseInt();     
  }
  Serial.println(setpoint);  */
  PIDcalculation();// find PID value  
  
  if (angle < setpoint) {
    digitalWrite(dir, LOW);// Forward motion
    //digitalWrite(dir, HIGH);
  }
  else {
    digitalWrite(dir, HIGH);//Reverse motion
    //digitalWrite(dir2, LOW);
  }
   
  Serial.println(pidTerm_scaled);
  
  analogWrite(pwm, pidTerm_scaled); 

  //Serial.println("WHEEL ANGLE:");
  
  delay(100);
}

void PIDcalculation(){
  int pin1 = analogRead(A0);  
  int angle ;
  angle = map(pin1,0,1023,0,360); //count to angle conversion  
  Serial.println(angle);  
  error = setpoint - angle;  
  changeError = error - last_error; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255); //constraining to appropriate value
  pidTerm_scaled = abs(pidTerm); //to make sure it's a positive value
  last_error = error;
}
  
/*void Achange() //these functions are for finding the encoder counts
{
  A = digitalRead(2);
  B = digitalRead(3);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;
  switch (state)
  {
    case 1:
    {
      if (statep == 2) count++;
      if (statep == 4) count--;
      break;
    }
    case 2:
    {
      if (statep == 1) count--;
      if (statep == 3) count++;
      break;
    }
    case 3:
    {
      if (statep == 2) count --;
      if (statep == 4) count ++;
      break;
    }
    default:
    {
      if (statep == 1) count++;
      if (statep == 3) count--;
    }
  }
  statep = state;

}

void Bchange()
{
  A = digitalRead(2);
  B = digitalRead(3);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;
  switch (state)
  {
    case 1:
    {
      if (statep == 2) count++;
      if (statep == 4) count--;
      break;
    }
    case 2:
    {
      if (statep == 1) count--;
      if (statep == 3) count++;
      break;
    }
    case 3:
    {
      if (statep == 2) count --;
      if (statep == 4) count ++;
      break;
    }
    default:
    {
      if (statep == 1) count++;
      if (statep == 3) count--;
    }
  }
  statep = state;
 */ 



