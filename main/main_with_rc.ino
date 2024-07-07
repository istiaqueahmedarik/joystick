#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <SabertoothSimplified.h>
#include <Servo.h>
#include <std_msgs/Int64.h>
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <sbus.h>

#define ST_ADDRESS 128 // Sabertooth address (default: 128)
#define MOTOR1 1       // Sabertooth motor 1 //right
#define MOTOR2 2       // Sabertooth motor 2 //left

Servo boxservo1;
Servo boxservo2;
Servo pan;
Servo tilt;
int panpos1 = 90;
int tiltpos1 = 0;
int panpos = 0;
int tiltpos = 0;

SabertoothSimplified ST(Serial5);
SabertoothSimplified ST_ARM(Serial4);

bfs::SbusRx sbus_rx(&Serial2); /* SBUS object, writing SBUS */
bfs::SbusData data;
bool failsafe;

// Constants
const int STEPS_PER_REV = 1600; // Number of steps per revolution of the stepper motor
const float GEAR_RATIO = 50.0;  // Gear ratio of the stepper motor's gear box

// Function to calculate the wheel angle from the encoder count
float calculateWheelAngle(long count)
{
    float steps = static_cast<float>(count) / GEAR_RATIO; // Adjust the count with the gear ratio
    float angle = (steps / STEPS_PER_REV) * 360.0;        // Calculate the wheel angle in degrees
    return angle;
}

MultiStepper stepperss;
MultiStepper steppers;

long thrs_f = -10000;
long p = 0;
long thrs_b = 10000;
long thrs_f_m2 = -43000;
long thrs_b_m2 = 43000;
// int interrupt[] = {0,0,0,0,3,0,2,0};

const int max_speed = 100;
int speed = 50;
int gear = 50;
int step = 1600;

// arm
// base 10 11
// left right 6 7 8 9
// 4 5 gripper
//
long base_angle = 0;
long base_step;
long thrs_base_f = 100000;
long thrs_base_b = -100000;
int n = 0;

long base_ratio = 1000;
int z = 0;

long base;
long b = 0;
long l = 0;
long q = 0;
long x = 0;
long m = 0;
long y = 0;

int chVal[30] = {};

AccelStepper stepperB(1, 34, 33); // pul // dir //base  chVal[9]   chVal[1 and 2] sabertooth
AccelStepper stepperL(1, 36, 35); // science L
AccelStepper stepperR(1, 38, 37); // science R // uporer ta

AccelStepper stepperG(1, 40, 39); // gripper// gripper
// arm

long positionss[] = {0, 0, 0, 0};

long a = 0;

const int tt = 50;

long positions[2];

const int t = 200;

ros::NodeHandle nh;
unsigned long lastJoyStickCtrlTime = 0;
int mode = 1;
bool r_mode = false;
bool wifi = false;
// Callback function to handle incoming joystick messages
void box1(int panval)
{

    if (panval == 1000)
    {

        boxservo1.write(0);
    }

    else if (panval == 2000)
    {

        boxservo1.write(180);
    }
}

void box2(int tiltval)
{

    if (tiltval == 2000)
    {
        tiltpos++;

        boxservo2.write(180);
    }

    else if (tiltval == 1000)
    {

        boxservo2.write(0);
    }
}
void panServo(int panval)
{

    if (panval == 1000)
    {
        panpos1 -= 3;
        panpos1 = max(0, panpos1);
        pan.write(panpos1);
    }

    else if (panval == 2000)
    {
        panpos1 += 3;
        panpos1 = min(90, panpos1);

        pan.write(panpos1);
    }
}

void tiltServo(int tiltval)
{

    if (tiltval == 2000)
    {
        tiltpos1 += 3;
        tiltpos1 = min(180, tiltpos1);

        tilt.write(tiltpos1);
    }

    else if (tiltval == 1000)
    {
        tiltpos1 -= 3;
        tiltpos1 = max(0, tiltpos1);

        tilt.write(tiltpos1);
    }
}
void joystickCallback(const std_msgs::String &msg)
{
    nh.spinOnce();
    lastJoyStickCtrlTime = millis();
    // Access the joystick data from the message

    std::vector<int> arr;
    // fl.data = msg.data;
    std::string input = msg.data;
    std::istringstream iss(input.substr(1, input.length() - 2)); // Remove the enclosing brackets

    int num;
    char delimiter;
    while (iss >> num)
    {
        arr.push_back(num);
        iss >> delimiter;
    }

    // // vl.data = arr[0];
    // float yu = arr[3];
    // float yl = arr[2];
    // // Set motor 1 to move forward at full speed

    int SPEED = 80;
    if (arr.size() >= 9)
    {
        if (arr[8] == 2000)
            SPEED = 70;
    }

    ST.motor(MOTOR2, constrain(map(arr[0], 1000, 2000, -SPEED, SPEED), -SPEED, SPEED));
    ST.motor(MOTOR1, constrain(map(arr[1], 1000, 2000, -SPEED, SPEED), -SPEED, SPEED));

    if (arr.size() >= 4)
    {
        ST_ARM.motor(1, constrain(map(arr[2], 1000, 2000, 100, -100), -100, 100));
        ST_ARM.motor(2, constrain(map(arr[3], 1000, 2000, 100, -100), -100, 100));
    }
    if (arr.size() >= 5)
    {

        chVal[9] = arr[4];
    }
    if (arr.size() >= 6)
    {

        chVal[10] = arr[5];
    }
    if (arr.size() >= 7)
    {

        chVal[5] = arr[6];
    }

    if (arr.size() >= 8)
    {
        if (arr[7] == 1500)
        {
            mode = 1;
        }
        else if (arr[7] == 1000)
        {
            mode = 2;
        }
        else if (arr[7] == 2000)
        {
            mode = 3;
        }
    }
    if (arr.size() >= 14)
    {
        box1(arr[13]);
    }
    if (arr.size() >= 15)
    {
        box2(arr[14]);
    }
    if (arr.size() >= 12)
    {
        panServo(arr[11]);
    }
    if (arr.size() >= 13)
    {
        tiltServo(arr[12]);
    }
    // // Set motor 2 to move backward at full speed
    // ST.motor(MOTOR2, yl);
    // vl.data = yu;
    // pub.publish(&msg);

    // Publish the new message
    nh.spinOnce();
    delay(0.01);
}
void wifiCallback(const std_msgs::String &msg)
{
    if (msg.data == "connected")
    {
        wifi = true;
    }
    else
    {
        wifi = false;
    }
}
ros::Subscriber<std_msgs::String> joystickSub("joystick", &joystickCallback);
ros::Subscriber<std_msgs::String> wifiSub("wifi", &wifiCallback);

void setup()
{

    nh.getHardware()->setBaud(115200);

    // int timeout_ms = 100;

    // Serial.setTimeout(timeout_ms);

    Serial5.begin(9600);
    Serial4.begin(9600);
    Serial.begin(9600);
    // Serial.begin(115200);
    sbus_rx.Begin();

    boxservo1.attach(3);
    boxservo2.attach(2);
    boxservo1.write(panpos);
    boxservo2.write(tiltpos);
    // 11
    pan.attach(4);
    tilt.attach(5);
    pan.write(panpos1);
    tilt.write(tiltpos1);
    stepper_setup();
    positions[0] = 0;
    positions[1] = 0;

    nh.initNode();
    nh.subscribe(joystickSub);
    // nh.subscribe(pxstateSub);
    // nh.subscribe(vstateSub);
    // nh.advertise(vals);
    // nh.advertise(pub);

    nh.spinOnce();
    delay(0.5);
}

void handleUtils()
{
    HANDLE_LIFTER();

    base_arm(); // ch 4
    gripper();  // ch

    nh.spinOnce();
    delay(0.01);
}

void loop()
{
    if (!nh.connected() && failsafe) // serial off and failsafe on
    {
        ST.motor(MOTOR1, 0);
        ST.motor(MOTOR2, 0);
        ST_ARM.motor(MOTOR1, 0);
        ST_ARM.motor(MOTOR2, 0);
    }
    else if (!nh.connected() && !failsafe) // serial off and failsafe off
    {
        Serial.println("Serial off");
        get_sbus();
    }
    else if (nh.connected() && r_mode)
    {
        get_sbus();
    }
    else if (nh.connected() && !wifi)
    {
        get_sbus();
    }
    handleUtils();

    // Add additional code here if needed
}

void stepper_setup()
{
    steppers.addStepper(stepperL);
    steppers.addStepper(stepperR);
    stepperR.setMaxSpeed(1000);
    stepperR.setAcceleration(500);
    stepperL.setMaxSpeed(1000);
    stepperL.setAcceleration(500);
    stepperR.setSpeed(300);
    stepperL.setSpeed(300);

    stepperB.setMaxSpeed(900000);
    stepperB.setAcceleration(1000);
    stepperB.setSpeed(900000);

    stepperB.setMinPulseWidth(t);
    stepperL.setMinPulseWidth(t);
    stepperR.setMinPulseWidth(t);

    stepperG.setMaxSpeed(900000);
    stepperG.setAcceleration(500);
    stepperG.setSpeed(900000);
    stepperG.setMinPulseWidth(t);
    nh.spinOnce();
    delay(0.01);
}

void print_stepper_pos()
{

    nh.spinOnce();
    delay(0.01);
}

void base_arm()
{

    if ((chVal[9] < 1400 || chVal[9] > 1600) && stepperB.currentPosition() <= thrs_base_f && stepperB.currentPosition() >= thrs_base_b)
    {

        if (chVal[9] > 1600 && stepperB.currentPosition() <= thrs_base_f && chVal[9] < 2100)
            b += 500;
        else if (chVal[9] < 1400 && stepperB.currentPosition() >= thrs_base_b && chVal[9] > 900)
            b -= 500;

        stepperB.moveTo(b);
        stepperB.setSpeed(5000);
        stepperB.runSpeedToPosition();
        // Serial.println(base_step);
    }

    else
    {
        b = 0;
        // positions[0]=0;
        // positions[1]=0;
    }
    if (stepperB.currentPosition() >= thrs_base_f)
        positions[0] = thrs_base_f;
    if (stepperB.currentPosition() <= thrs_base_b)
        positions[0] = thrs_base_b;
    nh.spinOnce();
    delay(0.01);
}

void fix()
{
    // chVal[5]=0
    positions[0] = 2400;
    positions[1] = 2400;

    steppers.moveTo(positions);

    steppers.runSpeedToPosition();

    stepperL.setCurrentPosition(0);
    stepperR.setCurrentPosition(0);
    nh.spinOnce();
    delay(0.01);
}

void HANDLE_LIFTER()
{

    if (mode == 1)
    {
        stepperR.setCurrentPosition(0);
        stepperL.setCurrentPosition(0);
        z = 0;
        if (chVal[10] > 1600 && chVal[10] < 2100)
        {
            z += 50;
            Serial.println("-->");
        }
        else if (chVal[10] < 1400 && chVal[10] > 900)
        {
            {
                z -= 50;
                Serial.println("-->");
            }
        }
        // else z =0;
        positions[0] = z;
        positions[1] = z;

        steppers.moveTo(positions);

        steppers.runSpeedToPosition();
        // stepperR.setCurrentPosition(0);
        //  stepperL.setCurrentPosition(0);
    }
    if (mode == 2)
    {

        stepperR.setCurrentPosition(0);
        stepperL.setCurrentPosition(0);
        x = 0;
        if (chVal[10] < 1450 && x > -12000)
            x -= 50;
        if (chVal[10] > 1550 && x < 12000)
            x += 50;

        positions[0] = x;
        positions[1] = -x;

        steppers.moveTo(positions);
        steppers.runSpeedToPosition();
        //  stepperR.setCurrentPosition(0);
        //  stepperL.setCurrentPosition(0);
    }
    nh.spinOnce();
    delay(0.01);
}
void gripper()
{
    stepperG.setCurrentPosition(0);
    m = 0;
    if (chVal[5] > 1600 && chVal[5] < 2100)
        m -= 500;
    else if (chVal[5] < 1400 && chVal[5] > 900)
        m += 500;
    // //else z =0;

    stepperG.moveTo(m);

    stepperG.moveTo(m);
    stepperG.setSpeed(5000);
    stepperG.runSpeedToPosition();
    nh.spinOnce();
    delay(0.01);
}

void get_sbus()
{
    if (sbus_rx.Read())
    {
        /* Grab the received data */
        data = sbus_rx.data();
        /* Display the received data */

        chVal[1] = map(data.ch[0], 210, 1750, 950, 2000);
        chVal[2] = map(data.ch[1], 210, 1750, 950, 2000);
        chVal[3] = map(data.ch[2], 210, 1750, 950, 2000);
        chVal[4] = map(data.ch[3], 210, 1750, 950, 2000);
        chVal[5] = map(data.ch[4], 210, 1750, 950, 2000);
        chVal[6] = map(data.ch[5], 210, 1750, 950, 2000);
        chVal[7] = map(data.ch[6], 210, 1750, 950, 2000);
        chVal[8] = map(data.ch[7], 210, 1750, 950, 2000);
        chVal[9] = map(data.ch[8], 210, 1750, 950, 2000);
        chVal[10] = map(data.ch[9], 210, 1750, 950, 2000);
        chVal[11] = map(data.ch[10], 210, 1750, 950, 2000);
        chVal[12] = map(data.ch[11], 210, 1750, 950, 2000);
        chVal[13] = map(data.ch[12], 210, 1750, 950, 2000);
        chVal[14] = map(data.ch[13], 210, 1750, 950, 2000);
        chVal[15] = map(data.ch[14], 210, 1750, 950, 2000);
        chVal[16] = map(data.ch[15], 210, 1750, 950, 2000);

        failsafe = data.failsafe;

        // printall();
    }
}
