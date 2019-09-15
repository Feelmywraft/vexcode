/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/



#ifndef VEX_H
#define VEX_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v5.h"
#include "v5_vcs.h"
#define TOP -2390.4


//Brain
vex::brain       Brain;

//Left Drive Train
 vex::motor Motor1 = vex::motor(vex::PORT10,vex::gearSetting::ratio18_1,false);

vex::motor Motor2 = vex::motor(vex::PORT17,vex::gearSetting::ratio18_1,false);


//Right Drive Train
vex::motor Motor3 = vex::motor(vex::PORT6,vex::gearSetting::ratio18_1,true);

vex::motor Motor4 = vex::motor(vex::PORT13,vex::gearSetting::ratio18_1,true);


//Intake Spinners
vex::motor Motor5 = vex::motor(vex::PORT9,vex::gearSetting::ratio6_1);

vex::motor Motor6 = vex::motor(vex::PORT7,vex::gearSetting::ratio6_1);


//Intake Raise/Decend
vex::motor Motor7 = vex::motor(vex::PORT5,vex::gearSetting::ratio6_1);


//Tray
vex::motor Motor8 = vex::motor(vex::PORT2,vex::gearSetting::ratio6_1);


//Controller
vex::controller Controller1 = vex::controller();

//Gyro
vex::gyro gyro1 = vex::gyro(Brain.ThreeWirePort.A);


void MacroStack(){
  Motor8.rotateTo(TOP, vex::rotationUnits::deg, 15, vex::velocityUnits::pct);
}

void turning(int targetangle, int speed){
  if(targetangle<gyro1.value(vex::rotationUnits::deg)){
    while(targetangle<gyro1.value(vex::rotationUnits::deg)){
      Motor1.spin(vex::directionType::rev, 50, vex::percentUnits::pct);
      Motor2.spin(vex::directionType::rev, 50, vex::percentUnits::pct);
      Motor3.spin(vex::directionType::fwd, 50, vex::percentUnits::pct);
      Motor4.spin(vex::directionType::fwd, 50, vex::percentUnits::pct);
    }
  }else{
while(targetangle>gyro1.value(vex::rotationUnits::deg)){
      Motor3.spin(vex::directionType::rev, 50, vex::percentUnits::pct);
      Motor4.spin(vex::directionType::rev, 50, vex::percentUnits::pct);
      Motor1.spin(vex::directionType::fwd, 50, vex::percentUnits::pct);
      Motor2.spin(vex::directionType::fwd, 50, vex::percentUnits::pct);
    }

  }
  Motor1.stop(vex::brakeType::hold);
  Motor2.stop(vex::brakeType::hold);
  Motor3.stop(vex::brakeType::hold);
  Motor4.stop(vex::brakeType::hold);

}

class PID2{
double currentvalue,target,kp,ki,kd,error,preverror,output,integral,derivative;

public:
    PID2(double kp,double kd,double ki,double target){
      this->kp = kp;
      this->kd = kd;
      this->ki = ki;
      this->target = target;
    }
    double calculate_error(double current){
      return target-current;
    }
    double get_error(){
      return this->error;
    }
    double calculate_power(double sensorValue){
      this->error = calculate_error(sensorValue);
      this->derivative = (error-preverror);
      this->integral += this->error;

      double power = this->error*this->kp+this->derivative*this->kd+this->integral*this->ki;
      this->preverror =error;
      return power;
    }


};

#endif