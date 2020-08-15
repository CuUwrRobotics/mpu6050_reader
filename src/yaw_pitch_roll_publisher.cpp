#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"

#include "ros/ros.h"
#include "custom_sensor_msgs/ypr.h" // This should be sperated

#include "MPU6050.cpp"
#include "I2Cdev.cpp"

MPU6050 mpu;

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
//#define OUTPUT_READABLE_YAWPITCHROLL


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

ros::Publisher gyroPos;

#define ROS_TOPIC_NAME "gyro_pos"
#define RADIAN_OUTPUT true

#define PREV_VALUES 100 // How many values to track, max 254
#define SAFE_CHANGE_THRESH .1 // Maximum deviation in values while settling
#define SAFE_CHANGE_THRESH_RAD .005 // Maximum deviation in values while settling (radians)
#define STAY_SAFE_COUNT 500 // How many reads should be 'safe'
float previousValues[2][PREV_VALUES];
int safeCount = 0;
bool dataGood = false;

float calcRange(float data[], uint8_t count) {
    float min = data[0], max = data[0];
    for(uint8_t i = 0; i < count; i++) { 
        if(data[i] < min)
            min = data[i];
        else if (data[i] > max)
            max = data[0];
    }

    return max - min;
}

void appendArray(float *array, uint8_t arraySize, float data){
    for(int i = arraySize - 1; i > 0; i--)
        array[i] = array[i - 1]; // Shift data in array
    array[0] = data;
}

void setup() {
    // initialize device
    ROS_INFO("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    ROS_INFO("Testing device connections...\n");
    ROS_INFO(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    ROS_DEBUG("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        ROS_DEBUG("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        ROS_DEBUG("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        ROS_ERROR("DMP Initialization failed (code %d)\n", devStatus);
    }
}

void loop() {
    usleep(1000);
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        ROS_INFO("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 10) {
        // read a packet from FIFO
        int fifoReturn = 0;
        fifoReturn = mpu.getFIFOBytes(fifoBuffer, packetSize);
        if(fifoReturn < 0){ // Notify on communication error
            dataGood = false;
            safeCount = 0;
        }

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if(!RADIAN_OUTPUT) {
            ypr[0] *= 180/M_PI;
            ypr[1] *= 180/M_PI;
            ypr[2] *= 180/M_PI;
        }

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            ROS_INFO("ypr  %7.2f %7.2f %7.2f    ", ypr[0], ypr[1], ypr[2]);
            ROS_INFO("\n");
        #endif
        if(!dataGood){ // Checks that data is within safe thresholdss
            appendArray(previousValues[0], PREV_VALUES, ypr[1]);
            appendArray(previousValues[1], PREV_VALUES, ypr[2]);
            if((!RADIAN_OUTPUT) && calcRange(previousValues[0], PREV_VALUES) <= SAFE_CHANGE_THRESH 
                && calcRange(previousValues[1], PREV_VALUES) <= SAFE_CHANGE_THRESH)
                safeCount++; // Data safe; add to counter
            else if((RADIAN_OUTPUT) && calcRange(previousValues[0], PREV_VALUES) <= SAFE_CHANGE_THRESH_RAD 
                && calcRange(previousValues[1], PREV_VALUES) <= SAFE_CHANGE_THRESH_RAD)
                safeCount++; // Data safe; add to counter
            else 
                safeCount = 0; // Data left safe threshold; reset counter

            if (safeCount > STAY_SAFE_COUNT) { // When enough safe reading have occurred
                dataGood = true; // Say data is safe
                ROS_INFO("Data stable!");
            }
        }

        if(ros::ok()) {
            custom_sensor_msgs::ypr msg;
            msg.dataGood = dataGood;
            msg.inRadians = RADIAN_OUTPUT;
            msg.y = ypr[0];
            msg.p = ypr[1];
            msg.r = ypr[2];
            gyroPos.publish(msg);
            ros::spinOnce();
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mpu6050_reader");
    ros::NodeHandle n;
    gyroPos = n.advertise<custom_sensor_msgs::ypr>(ROS_TOPIC_NAME, 1);

    setup();
    usleep(100000);
    ROS_INFO("Writing data to topic: /%s", ROS_TOPIC_NAME);
    ROS_INFO("Waiting for data stability...");
    for (;;)
        loop();

    return 0;
}

