// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <cstdlib>
#include <stdio.h>
#include <pigpio.h>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

 int m1_A = 13; //Motor 1 forward
 int m1_B = 19; //Motor 1 reverse
 int m1_E = 24; //Motor 1 enable
 int m2_A = 20; //Motor 2 forward
 int m2_B = 21; //Motor 2 reverse
 int m2_E = 23; //Motor 2 enable

 int servo_pin = 18; // Steering pin

 typedef struct
 {
     int gpio;
     int minPulse;
     int maxPulse;
     int pw;
     int pwInc;
     int connected;
 } servoInf_t;

 // Initialize servo information
 servoInf_t servoInf = {18, 1000, 2000, 1500,  29, 1};

 // Encoder information
 // CLK Pin left motor encoder
 int enc_left_motor_clk = 14; //Motor 1 reverse
 // DT pin left motor encoder
 int dt_left_motor_clk = 15;
 // + voltage left motor encoder
 // GND pin

 void setPinDirections();
 void moveMotorsForward(int pwm_fill);
 void moveMotorsBackward(int pwm_fill);
 void stopMotors();

 int main(int argc, char *argv[])
 {
     if (gpioInitialise() < 0)
     {
         return 1;
     }

     setPinDirections();

     // Position servo in the middle
     gpioServo(servoInf.gpio, servoInf.pw);

     int pwm_fill = 150;
     std::string str;
     char pressedKey;

     while(true)
     {
         cv::Mat disp = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);
         pressedKey = cv::waitKey(1);

         switch (pressedKey)
         {
             case 'w': // Forward
                 pwm_fill += 25;
                 if(pwm_fill > 255)
                 {
                     pwm_fill = 255;
                 }
                 if(pwm_fill > 0)
                 {
                     moveMotorsForward(pwm_fill);
                 }
                 else
                 {
                     moveMotorsBackward(-pwm_fill);
                 }
                 break;
             case 's': // Backward
                 pwm_fill -= 25;
                 if(pwm_fill < -255)
                 {
                     pwm_fill = -255;
                 }
                 if(pwm_fill > 0)
                 {
                    moveMotorsForward(pwm_fill);
                 }
                 else
                 {
                    moveMotorsBackward(-pwm_fill);
                 }
                 break;
             case 'x':
                 stopMotors();
                 pwm_fill = 0;
                 break;
             case 'a':
                 servoInf.pw -= servoInf.pwInc;
                 if(servoInf.pw < servoInf.minPulse)
                 {
                     servoInf.pw = servoInf.minPulse;
                 }
                 gpioServo(servoInf.gpio, servoInf.pw);
                 break;
             case 'd':
                 servoInf.pw += servoInf.pwInc;
                 if(servoInf.pw > servoInf.maxPulse)
                 {
                     servoInf.pw = servoInf.maxPulse;
                 }
                 gpioServo(servoInf.gpio, servoInf.pw);
                 break;
             default:
                 break;
        }

        str = "Current motor pwm(0-255): " + std::to_string(pwm_fill) + ";";
        cv::putText(disp, str, cv::Point(20, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        str = "Steering pw: " + std::to_string(servoInf.pw) + ";";
        cv::putText(disp, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "Commands:", cv::Point(20, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'w' - Forward", cv::Point(20, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'s' - Backward", cv::Point(20, 110), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'a' - Left", cv::Point(20, 130), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'d' - Right", cv::Point(20, 150), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'x' - Stop", cv::Point(20, 170), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);

        cv::imshow("RasPi control test", disp);
    }
 }

 void setPinDirections()
 {
     gpioSetMode(m1_A, PI_OUTPUT);
     gpioSetMode(m1_B, PI_OUTPUT);
     gpioSetMode(m1_E, PI_OUTPUT);
     gpioSetMode(m2_A, PI_OUTPUT);
     gpioSetMode(m2_B, PI_OUTPUT);
     gpioSetMode(m2_E, PI_OUTPUT);
 }

 void moveMotorsForward(int pwm_fill)
 {
     gpioWrite(m1_A, 1);
     gpioWrite(m1_B, 0);
     gpioWrite(m2_A, 1);
     gpioWrite(m2_B, 0);
     gpioPWM(m1_E, pwm_fill);
     gpioPWM(m2_E, pwm_fill);
 }

 void moveMotorsBackward(int pwm_fill)
 {
     gpioWrite(m1_A, 0);
     gpioWrite(m1_B, 1);
     gpioWrite(m2_A, 0);
     gpioWrite(m2_B, 1);
     gpioPWM(m1_E, pwm_fill);
     gpioPWM(m2_E, pwm_fill);
 }

 void stopMotors()
 {
     gpioPWM(m1_E, 0);
     gpioPWM(m2_E, 0);
 }