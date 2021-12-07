#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int wheelSpd = 150;

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11;
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

const int minValues[8] = { 645, 552, 506, 483, 506,  529, 483, 740 };
const int maxValues[8] = { 1687, 1255, 1610, 1088, 1279, 1399, 1325, 1760 };

//want max speed of wheel to be 2 times max
//const float k_p = wheelSpd/1000.0;
const float k_p = wheelSpd/1500.0;
const float k_d = 1.4;

bool hasTurnedAround = false;

int prevError = 0;

int turnTime = 0;
///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
}

bool checkTurnAround()
{
  int sum = 0;
  for(unsigned char i = 0; i < 8; i++)
  { 
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
    if(sensorValues[i] < 1500)
      return false;
    sum+=sensorValues[i];
  }
  if(sum == 2500*8)
    return false;
  //put car into turning around state
  //don't want to worry about faulty sensor recording while turning around
  return true;
}

void turnAround()
{
  digitalWrite(left_dir_pin,HIGH);
  digitalWrite(right_dir_pin,HIGH);
  delay(120);
  digitalWrite(right_dir_pin,LOW);
//  digitalWrite(left_dir_pin,HIGH);
  
  //should be done turning around by this point
  turnTime = millis();
  if(hasTurnedAround) //stop the car
  {
    digitalWrite(left_nslp_pin,LOW);
    digitalWrite(right_nslp_pin,LOW);
  }
  analogWrite(left_pwm_pin,60);
  analogWrite(right_pwm_pin, 60);
  delay(906);
  //set wheel direction back to normal
  digitalWrite(left_dir_pin,LOW);
  
  analogWrite(left_pwm_pin,wheelSpd);
  analogWrite(right_pwm_pin, wheelSpd);
//  delay(100);
  hasTurnedAround = true;

}

void loop() {
  // put your main code here, to run repeatedly: 
    int leftSpd = wheelSpd;
    int rightSpd = wheelSpd;
    int k[8];
    if(millis() < 2400)
    {
      leftSpd = 100;
      rightSpd = 100;
    }

  ECE3_read_IR(sensorValues);


  if(checkTurnAround())
  {
    turnAround();
  }

    
//  Serial.println();
//  delay(250);

  for(unsigned char i = 0; i < 8; i++)
  { 
    k[i] = sensorValues[i] - minValues[i];

    
    if(k[i] <= 0)
    {
      k[i] = 0;
    }
    k[i] = k[i]*1000/maxValues[i];
  }

  //gives us our danger values. Closer to zero means less dangerous
  //if value is negative need to turn right
  //if value is positive need to turn left
  int errorValue = -8*k[0] - 4*k[1] - 2*k[2] - k[3] + k[4] + 2*k[5] + 4*k[6] + 8*k[7];
  errorValue = errorValue/4;
  
  int changeInError = errorValue - prevError;
  
  //update values for next iteration
  prevError = errorValue;
  
  //left speed is subtracted since errorValue will be negative when turning right
  leftSpd = leftSpd - k_p*errorValue - k_d*changeInError;
  rightSpd = rightSpd + k_p*errorValue + k_d*changeInError;
  leftSpd = min(leftSpd, 254);
  rightSpd = min(rightSpd, 254);
  
    analogWrite(left_pwm_pin,leftSpd);
    analogWrite(right_pwm_pin, rightSpd);
    
  }
