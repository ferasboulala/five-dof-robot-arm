#include <joystick.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

Servo servo_1_1, servo_2_2, servo_3_1, servo_4_1, claw;
float servo_position[4] = {0, 0, 0, 0};
bool moving = false;

Joystick *joystickTeal = new Joystick(2,0,1);
Joystick *joystickBlue = new Joystick(3,2,3);
Joystick *joystickWhite = new Joystick(4,4,5);
Joystick *joystickYellow = new Joystick(5,6,7);

bool record = false;
bool stopRecord = true;
bool play = false;

int current = 0;

const int MAX_SEQUENCE = 25;

int sequence[MAX_SEQUENCE][2];
int positionBeforeSequence[5] = {90, 100, 10, 90, 45};

Servo servos[5] = {servo_1_1, servo_2_2, servo_3_1, servo_4_1, claw};

//std_msgs::Float64 str_msg;

void attachServos(){
  servo_1_1.attach(6); 
  servo_2_2.attach(7); 
  servo_3_1.attach(8); 
  servo_4_1.attach(9);
  claw.attach(10);
}

void neutralPosition(){
  servo_1_1.write(90); 
  servo_2_2.write(100); 
  servo_3_1.write(10);
  servo_4_1.write(90);
  claw.write(45);
}

float radiansToDegrees(float& radian){
 return (radian/3.1415)*180; 
}

//void velocityCallback(const std_msgs::Float64 &msg){
//  if (msg.data){
//    moving = true;
//  }
//  else{
//    moving = false;
//  }
//}

void callback1(const std_msgs::Float64 &msg){
  servo_position[0] = msg.data;
}

void callback2(const std_msgs::Float64 &msg){
  servo_position[1] = msg.data;
}

void callback3(const std_msgs::Float64 &msg){
  servo_position[2] = msg.data;
}

void callback4(const std_msgs::Float64 &msg){
  servo_position[3] = msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64> sub1("servo_1_1_position", callback1);
ros::Subscriber<std_msgs::Float64> sub2("servo_2_2_position", callback2);
ros::Subscriber<std_msgs::Float64> sub3("servo_3_1_position", callback3);
ros::Subscriber<std_msgs::Float64> sub4("servo_4_1_position", callback4);
//ros::Subscriber<std_msgs::Float64> sub5("arm_velocity", velocityCallback);

//ros::Publisher chatter("chatter", &str_msg);

void subscribeToAll(){
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
}

void setup(){
  nh.getHardware()->setBaud(115200);  
  nh.initNode();
  subscribeToAll();
  attachServos();
  neutralPosition();
  
  // Switch which will enable ROS:
  pinMode(12, INPUT);
  digitalWrite(12, HIGH);
  
  sequence[0][0] = 0;
  sequence[0][1] = 90;
  
  Serial.print("Finished setting up \n");
  delay(1000);
}

void moveArm(){
  if (!moving) {
    float current[4] = {servo_1_1.read(), servo_2_2.read(), servo_3_1.read(), servo_4_1.read()};
    for (int i = 0; i < 4; i++){
      if (true){
//        servo_1_1.write(radiansToDegrees(servo_position[0]));
//        delay(30);
        servo_2_2.write(90+radiansToDegrees(servo_position[1]));
        delay(40);
        servo_3_1.write(120+radiansToDegrees(servo_position[2]));
        delay(40);
        servo_4_1.write(radiansToDegrees(servo_position[3]));
      }
    }
  }
}

void playSequence() {
  for (int i = 0; i < 5; i++){
    servoSweep(servos[i], servos[i].read(), positionBeforeSequence[i]);
  }
  delay(1000);
  for (int i = 2; i < current; i = i + 2){
    servoSweep(servos[sequence[i][0]], sequence[i][1], sequence[i + 1][1]);
  }
}

void servoSweep(Servo &servoSweeping, int first, int last) {
  if (first >= last){
    for (int i = first; i >= last; i--){
      servoSweeping.write(i);
      delay(20);
    }
  }
  else{
    for (int i = first; i <= last; i++){
      servoSweeping.write(i);
      delay(20);
    }
  }
}

void loop(){
  //chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
  while (digitalRead(12)){
    // Values of joysticks range from 0 to 900, practically
    int servo_1_1X = joystickTeal->xValue(); // BASE
    int servo_2_2X = joystickBlue->xValue(); // SHOULDER
    int servo_3_1X = joystickWhite->xValue(); // ELBOW
    int servo_4_1X = joystickYellow->xValue(); // WRIST
    uint8_t clawZ = joystickYellow->zValue();
    uint8_t servo_1_1Z = joystickTeal->zValue();
    uint8_t servo_2_2Z = joystickBlue->zValue();
    uint8_t servo_3_1Z = joystickWhite->zValue();
  
    // servo_1_1: 0 to 180
    // servo_2_2: 10 to 170
    // servo_3_1: 30 to 120
    // servo_4_1: 10 to 170
    // CLAW: 10 to 80
    
    // Here:
    // stopRecord = true, record = false, play = false
    
    if (!servo_1_1Z && stopRecord && !record && !play) { // RECORDING PHASE //
      stopRecord = false;
      record = true;
      Serial.print("Started recording \n");
      for (int i = 0; i<5; i++){
        positionBeforeSequence[i] = servos[i].read();
        Serial.print(servos[i].read());
        Serial.print("\n");
      }
    }
    
    if(!servo_2_2Z && !stopRecord && record && !play){ // STOPPED PHASE //
      stopRecord = true;
      record = false;
      sequence[current + 1][0] = sequence[current][0];
      sequence[current + 1][1] = servos[sequence[current][0]].read();
      current++;
      Serial.print("Stopped recording \n");
    }
    
    if (!servo_3_1Z && !play && !record && stopRecord) { 
      play = true;
      Serial.print("Playing recording in 3... \n"); // PLAYING PHASE //
      delay(3000);
      playSequence();
    }
    
    if (!record && stopRecord && play) { 
      play = false;
      Serial.print("Done with recording.\n ");
    }
    
    ///////////////////////////////////////// 
    if (servo_2_2X >= 550) {
      int pos = servo_2_2.read();
      if (pos < 170) {
        servo_2_2.write(pos+1);
        if (record){
          if (sequence[current][0] != 1){
            sequence[current + 1][0] = sequence[current][0];
            sequence[current + 1][1] = servos[sequence[current][0]].read();
            sequence[current + 2][0] = 1;
            sequence[current + 2][1] = pos;
            current = current + 2;
          }
        }
        delay(100);
      }
    }
  
    if (servo_2_2X <= 300) {
      int pos = servo_2_2.read();
      if (pos > 10) {
        servo_2_2.write(pos-1);
        if (record){
          if (sequence[current][0] != 1){
            sequence[current + 1][0] = sequence[current][0];
            sequence[current + 1][1] = servos[sequence[current][0]].read();
            sequence[current + 2][0] = 1;
            sequence[current + 2][1] = pos;
            current = current + 2;
          }
        }
        delay(100);
      }
    }
  /////////////////////////////////////////  
   if (servo_3_1X >= 550) {
      int pos = servo_3_1.read();
      if (pos < 120) {
        servo_3_1.write(pos+1);
        if (record){
          if (sequence[current][0] != 2){
            sequence[current + 1][0] = sequence[current][0];
            sequence[current + 1][1] = servos[sequence[current][0]].read();
            sequence[current + 2][0] = 2;
            sequence[current + 2][1] = pos;
            current = current + 2;
          }
        }
        delay(20);
      }
    }
  
    if (servo_3_1X <= 300) {
      int pos = servo_3_1.read();
      if (pos > 30) {
        servo_3_1.write(pos-1);
        if (record){
          if (sequence[current][0] != 2){
            sequence[current + 1][0] = sequence[current][0];
            sequence[current + 1][1] = servos[sequence[current][0]].read();
            sequence[current + 2][0] = 2;
            sequence[current + 2][1] = pos;
            current = current + 2;
          }
        }
        delay(20);
      }
    }
  ///////////////////////////////////////// 
    if (servo_4_1X >= 550) {
      int pos = servo_4_1.read();
      if (pos < 180) {
        servo_4_1.write(pos+1);
        if (record){
          if (sequence[current][0] != 3){
            sequence[current + 1][0] = sequence[current][0];
            sequence[current + 1][1] = servos[sequence[current][0]].read();
            sequence[current + 2][0] = 3;
            sequence[current + 2][1] = pos;
            current = current + 2;
          }
        }
        delay(20);
      }
    }
  
    if (servo_4_1X <= 300) {
      int pos = servo_4_1.read();
      if (pos > 0) {
        servo_4_1.write(pos-1);
        if (record){
          if (sequence[current][0] != 3){
            sequence[current + 1][0] = sequence[current][0];
            sequence[current + 1][1] = servos[sequence[current][0]].read();
            sequence[current + 2][0] = 3;
            sequence[current + 2][1] = pos;
            current = current + 2;
          }
        }
        delay(20);
      }
    }
    /////////////////////////////////////////
    if (clawZ) {
      claw.write(90);
    }
    else {
      claw.write(30);
    }
  }
  moveArm();
}


