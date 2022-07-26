#include "Arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <ros/time.h>
#include<PID_v1.h>
 
#define l_en_a 18      /// motor encoder pin
#define l_en_b 19
#define r_en_a 2 //20
#define r_en_b 3 //21

#define l_plus 4       /// motor power   2.3.4.5 previous
#define l_minus 5
#define r_plus 6
#define r_minus 7

#define LOOPTIME 10
int updatenh = 0;

ros::NodeHandle  nh;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 1

float demandx=0;
float demandz=0;
unsigned long currentMillis;
unsigned long prevMillis;
float encoder0Prev;
float encoder1Prev;
float encoder0Diff;
float encoder1Diff;
double demand_speed_left;
double demand_speed_right;


double left_kp = 4.5, left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 4.5 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  


void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);

std_msgs::Int16 right_wheel_msg;
ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);

double pos_act_left = 0;                    
double pos_act_right = 0;


void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(left_wheel_pub);
  nh.advertise(right_wheel_pub);
  
  //pinMode(12, OUTPUT);         // for led
  pinMode(l_plus,OUTPUT);
  pinMode(l_minus,OUTPUT);
  pinMode(r_plus,OUTPUT);
  pinMode(r_minus,OUTPUT);

   rightPID.SetMode(AUTOMATIC);
   rightPID.SetSampleTime(1);
   rightPID.SetOutputLimits(-100, 100);
 
   leftPID.SetMode(AUTOMATIC);
   leftPID.SetSampleTime(1);
   leftPID.SetOutputLimits(-100, 100);
  
  //pinMode(l_en_a,INPUT_PULLUP);
  //pinMode(l_en_b,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(l_en_a), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(l_en_b), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r_en_a), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r_en_b), change_right_b, CHANGE);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    
      Serial.print(encoder0Pos);
      Serial.print("  ");
      Serial.println(encoder1Pos);
    
    prevMillis = currentMillis;
    demand_speed_left = demandx - (demandz*0.50);      // convert linear and angular vel to RPM of left and right wl
    demand_speed_right = demandx + (demandz*0.50);

    //demand_speed_left = ((2 * demandx) - (demandz*0.300))/(2*0.054);      // convert linear and angular vel to RPM of left and right wl  <<<<<<  SAME  LIKE ABOVE  >>>>>>
    //demand_speed_right = ((2 * demandx) + (demandz*0.300))/(2*0.054);

    encoder0Diff = encoder0Pos - encoder0Prev;           // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;

    pos_act_left = encoder0Pos;                   ///////////////////////////////////////////////////////////////////////////////
    pos_act_right = encoder1Pos;
    
    encoder0Prev = encoder0Pos;                          // Saving values
    encoder1Prev = encoder1Pos;
    
    left_setpoint = demand_speed_left*117.80;             //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*117.80;

    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;

    leftPID.Compute();
    rotate_left(left_output);
    rightPID.Compute();
    rotate_right(right_output);
  
  publishPos(LOOPTIME);     //------------->>>  PUBLISH POS
  
  if(updatenh>10){
     nh.spinOnce();            /////   publish pos
     updatenh = 0;
   }
   else{
     updatenh++;
   }
}

  //Serial.println(encoder0Pos);
  Serial.print(demandx);
  //Serial.print("  ");
  Serial.println(demandx);
 }

void rotate_left(int l_value) {
  
  if(l_value>0){
    //Max Voltage with 16V battery with 12V required  ->---(((( not need wrong )))
    //(12/16)*255 ~=190                               ->--(((( not need wrong )))
  Serial.println("l__called");
//  Serial.println(plus);
    int l_out = map(l_value, 1, 100, 1, 200);
    analogWrite(l_plus,l_out);
    analogWrite(l_minus,0);
  }
  else if(l_value<0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    int l_out = map(l_value, -1, -100, 1, 200);
    analogWrite(l_plus,0);
    analogWrite(l_minus,l_out);
  }
    else if(l_value == 0){
    analogWrite(l_plus,0);
    analogWrite(l_minus,0);
  }
}

void rotate_right(int r_value) {
  if(r_value>0){
    //Max Voltage with 16V battery with 12V required  ->---(((( not need wrong )))
    //(12/16)*255 ~=190                               ->--(((( not need wrong )))
  Serial.println("r__called");
//  Serial.println(plus);
    int r_out = map(r_value, 1, 100, 1, 200);    // first it was 190
    analogWrite(r_plus,r_out);
    analogWrite(r_minus,0);
  }
  else if(r_value<0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    int r_out = map(r_value, -1, -100, 1, 200);
    analogWrite(r_plus,0);
    analogWrite(r_minus,r_out);
  }
    else if(r_value == 0){
    analogWrite(r_plus,0);
    analogWrite(r_minus,0);
  }
}

void publishPos(double time) {
  left_wheel_msg.data = pos_act_left;
  right_wheel_msg.data = pos_act_right;
  left_wheel_pub.publish(&left_wheel_msg);
  right_wheel_pub.publish(&right_wheel_msg);
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void change_left_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(l_en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(l_en_b) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(l_en_b) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void change_left_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(l_en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(l_en_a) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(l_en_a) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}


void change_right_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(r_en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(r_en_b) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(r_en_b) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void change_right_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(r_en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(r_en_a) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(r_en_a) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}
