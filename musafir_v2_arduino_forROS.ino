
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
//#include <std_msgs/String.h>

#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;
geometry_msgs::Twist msg;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

std_msgs::Int16 leftEnc;
ros::Publisher pub_leftEnc("lwheel_ticks", &leftEnc);

std_msgs::Int16 rightEnc;
ros::Publisher pub_rightEnc("rwheel_ticks", &rightEnc);

//std_msgs::Float32 leftWheelVel;
//ros::Publisher pub_leftEncRate("lwheel_rate", &leftWheelVel);

//std_msgs::Float32 rightWheelVel;
//ros::Publisher pub_rightEncRate("rwheel_rate", &rightWheelVel);

//std_msgs::String str_msg;
//ros::Publisher dummy_odom("dummy_odom", &str_msg);

float vl;
float vr;

float WHEELBASE = 0.32 ;                 //m
long int TICKS_PER_REV = 5480 ;       
float WHEEL_DIAMETER_METER = 0.16 ;       //m
float DISTANCE_PER_TICK = (M_PI*WHEEL_DIAMETER_METER) / ((float)TICKS_PER_REV) ;

//ODOMETRY PARAMETERS
double x = 0 , y = 0 ; 
float theta = 0 , c_theta ;
float sr , sl ,mean_s ;

char base_link[] = "/base_link";
char odom[] = "/odom";

String dummy_string =" ";
//

unsigned long timeOutPreviousMillis = 0;  //Velocity Time Out.
int velocityTimeOut = 1500; //ms
unsigned long currentMillis = 0;

unsigned long previousMillis = 0;
double interval = 10; // in ms    

unsigned long pubPreviousMillis = 0;
int pubInterval = 100;

unsigned long accPreviousMillis = 0;
int accInterval = 3; //ms
double changeVelPerLoop = 0.025; // meter per second square
double windowExit = 0.031; // 0.1m/s before
double accVelL = 0 , accVelR = 0;
double accVelLTemp = 0 , accVelRTemp = 0;

#include "MusafirMotor.h"
MusafirMotor motorL(41, 42, 45);
MusafirMotor motorR(49, 48, 46);

#include <PID_v1.h>
struct motorParams {
  double kp;
  double ki;
  double kd;
};

motorParams motorPIDL;
motorParams motorPIDR;

double measuredVelLeft = 0, measuredVelRight = 0;
double pwmL = 0, pwmR = 0;
double requiredVelLeft = 0, requiredVelRight = 0;
// PID (&input, &output, &setpoint, kp, ki, kd, DIRECT/REVERSE)
PID pidL(&measuredVelLeft, &pwmL, &accVelL, 30 , 0, 0, DIRECT);
PID pidR(&measuredVelRight, &pwmR, &accVelR, 30, 0, 0, DIRECT);
//PID pidL(&measuredVelLeft, &pwmL, &requiredVelLeft, 30 , 0, 0, DIRECT);
//PID pidR(&measuredVelRight, &pwmR, &requiredVelRight, 30, 0, 0, DIRECT);
//      PID////

boolean pidActive = true;

//ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", twistCallBack);
void left_wheel_desired_vel(const std_msgs::Int16& rate)
{
  requiredVelLeft = rate.data; 
  requiredVelLeft = constrain(requiredVelLeft, -4 , 4);  
  //if(requiredVelLeft >= 0.01)
  timeOutPreviousMillis=  millis();
  pidActive = true;
}

void right_wheel_desired_vel(const std_msgs::Int16& rate1)
{ 
  requiredVelRight = rate1.data ;
  requiredVelRight = constrain(requiredVelRight, -4 , 4);
  //if(requiredVelRight >= 0.01)
  timeOutPreviousMillis= millis();
  pidActive = true;
}

ros::Subscriber <std_msgs::Int16> subRateL("lwheel_desired_vel",left_wheel_desired_vel );
ros::Subscriber <std_msgs::Int16> subRateR("rwheel_desired_vel",right_wheel_desired_vel );



#include <Encoder.h>

Encoder encL(19, 27);
Encoder encR(18, 25);
double encCurrL, encCurrR;
long int encOldL, encOldR;
double encRateL, encRateR;
int enc_left_sign = 1 ;
int enc_right_sign = 1 ;

void setup()
{
  
  Serial.begin(115200);
//  Wire.begin();        // join i2c bus (address optional for master) 
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(pub_leftEnc) ;
  nh.advertise(pub_rightEnc);
 // nh.advertise(dummy_odom)  ;
  //nh.advertise(pub_leftEncRate);
  //nh.advertise(pub_rightEncRate);
  //nh.subscribe(sub);
  nh.subscribe(subRateL);
  nh.subscribe(subRateR);

  initPID();

  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);
}

void loop()
{
  currentMillis = millis();
  if (int(currentMillis - previousMillis) >= interval) {   //SETTING PID, Encoder and The Navigator.

    previousMillis = currentMillis;
    encCurrL = enc_left_sign * encL.read();
    encCurrR = enc_right_sign * encR.read();
    
    encRateL = double(encCurrL - encOldL);
    sl = (encRateL/TICKS_PER_REV) * 3.142 * WHEEL_DIAMETER_METER ; 
    measuredVelLeft = sl * 100.0; // since interval is 10ms thats why i am multiplying it *100 
    
    encRateR = double(encCurrR - encOldR);
    sr = (encRateR/TICKS_PER_REV) * 3.142 * WHEEL_DIAMETER_METER ; 
    measuredVelRight = sr * 100.0; // since interval is 10ms thats why i am multiplying it *100

    //performing odometry calculations
    mean_s = (sl + sr) / 2;
    theta = ( (sr - sl) / WHEELBASE ) + theta;
    
    if (theta>6.283f)
    theta-=6.283;
    else if(theta<0)
    theta+=6.283;

    x = x + mean_s * cos(theta);
    y = y + mean_s * sin(theta);
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
  
    t.transform.translation.x = x;
    t.transform.translation.y = y;
  
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
    
    /*dummy_string = String(x) + " , " + String(y) + " , " + String(theta) ;
    int str_len = dummy_string.length() + 1; 
    char char_array[str_len];
    dummy_string.toCharArray(char_array, str_len);
    str_msg.data = char_array;
    dummy_odom.publish(&str_msg);*/ 
    // ending odometry
    measuredVelLeft = abs(measuredVelLeft);
    measuredVelRight = abs(measuredVelRight);
    /*Serial.print(String(measuredVelLeft));
    Serial.print(" , ");
    Serial.println(requiredVelLeft);*/
    encOldL = encCurrL;
    encOldR = encCurrR;
    if (pidActive) {
      pidL.Compute();
      pidR.Compute();
      
      motorL.setPWM(pwmL);
      motorR.setPWM(pwmR);
   //Serial.println(pwmL);
    }
    //velocityDrive(100,100);
  }

  if (currentMillis - pubPreviousMillis >= pubInterval) {   //SETTING PID, Encoder and The Navigator.

    pubPreviousMillis = currentMillis;
    
    leftEnc.data=encCurrL;
    rightEnc.data=encCurrR;
    
   // leftEncRate.data = encRateL;
    //rightEncRate.data = encRateR; 

    pub_leftEnc.publish(&leftEnc);
    pub_rightEnc.publish(&rightEnc);
    //pub_leftEncRate.publish(&leftEncRate);
   // pub_rightEncRate.publish(&rightEncRate);
  }
  
  if (currentMillis - timeOutPreviousMillis >= velocityTimeOut ) {  // Break Velocity after Certain Time.
    brakeLeftMotor(250);
    brakeRighttMotor(250);
    requiredVelLeft = 0;
    requiredVelRight = 0;
    accVelLTemp = 0;
    accVelRTemp = 0;
    pidActive = false;
  }
  
  if (long(currentMillis - accPreviousMillis) >= accInterval && pidActive == true ) {  // Break Velocity after Certain Time.
      double a = requiredVelLeft - accVelLTemp;
      if (a > windowExit) {
        accVelLTemp += changeVelPerLoop;
      }
      else if (a < -windowExit) {
       accVelLTemp -= changeVelPerLoop;
      }
      else {
        accVelLTemp = requiredVelLeft;
      }
   // Serial.print("acc: ");
  //  Serial.println(accVelL);
      double b = requiredVelRight - accVelRTemp;
      if (b > windowExit) {
        accVelRTemp += changeVelPerLoop;
      }
      else if (b < -windowExit) {
        accVelRTemp -= changeVelPerLoop;
      }
      else {
        accVelRTemp = requiredVelRight;
      }

      accPreviousMillis = currentMillis ;
      
      if(accVelLTemp >= 0)
        motorL.setDir(FORWARD);    
      else if(accVelLTemp < 0)
        motorL.setDir(BACKWARD);
        
      if(accVelRTemp >= 0)
        motorR.setDir(FORWARD);    
      else if(accVelRTemp < 0)
        motorR. setDir(BACKWARD);
       
      accVelL = abs(accVelLTemp);
      accVelR = abs(accVelRTemp);
  }
  nh.spinOnce();
}

void velocityDrive(int VL,int VR)
{
  if(VL<0)
  { 
    motorL.setDir(BACKWARD);
    motorL.setPWM(abs(VL));  
    }
  else
  { 
    motorL.setDir(FORWARD);
    motorL.setPWM(abs(VL));
    }
  if(VR<0)
  { 
    motorR.setDir(BACKWARD);
    motorR.setPWM(abs(VR));
    }
  else
  { 
    motorR.setDir(FORWARD);
    motorR.setPWM(abs(VR));
    }  
  }

  void brakeLeftMotor(int intensity)
{
  intensity = constrain(intensity, 0, 250); //Double Checks
  motorL.setDir(BRAKE);
  motorL.setPWM(intensity);

}
void brakeRighttMotor(int intensity)
{
  intensity = constrain(intensity, 0, 250); //Double Checks
  motorR.setDir(BRAKE);
  motorR.setPWM(intensity);
}

void initPID(void) {
  pidL.SetMode(MANUAL); // PID CONTROL OFF
  pidR.SetMode(MANUAL);
  pidL.SetSampleTime(interval); // sample time for PID
  pidR.SetSampleTime(interval);
  pidL.SetOutputLimits(0, 250); // min/max PWM
  pidR.SetOutputLimits(0, 250);
}
