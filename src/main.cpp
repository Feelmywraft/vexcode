/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\kevin.zhang                                      */
/*    Created:      Sun Sep 01 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen


// A global instance of vex::competition
vex::competition Competition;

// define your global instances of motors and other devices here


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

gyro1.startCalibration();
    while(gyro1.isCalibrating()){
      
    }
  PID2 gyro_pid(1,0,0,0);
  while(Motor1.rotation(rotationUnits::deg)<1000){
      double correction = gyro_pid.calculate_power(gyro1.value(rotationUnits::deg));
      
      Motor1.spin(vex::directionType::fwd, 50+correction, vex::percentUnits::pct);
      Motor2.spin(vex::directionType::fwd, 50+correction, vex::percentUnits::pct);
      Motor3.spin(vex::directionType::fwd, 50-correction, vex::percentUnits::pct);
      Motor4.spin(vex::directionType::fwd, 50-correction, vex::percentUnits::pct);
  }
}

/*--------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol( void ) {
  Motor7.setStopping(hold); 
  // User control code here, inside the loop
  //this is a comment
  Controller1.ButtonRight.pressed(MacroStack);
  while (1) {
      while (true) {  
        Controller1.Screen.setCursor(1,1);   //-2390.4
        Controller1.Screen.print("Tray : %f", Motor8.rotation(rotationUnits::deg));
        Controller1.Screen.newLine();
        Controller1.Screen.print("Intake : %f", Motor7.rotation(rotationUnits::deg));
             
        Motor1.spin(directionType::fwd, Controller1.Axis3.position(percentUnits::pct), velocityUnits::pct); //(Axis3+Axis4)/2;
        Motor2.spin(directionType::fwd, Controller1.Axis3.position(percentUnits::pct), velocityUnits::pct); //(Axis3+Axis4)/2;
        Motor3.spin(directionType::fwd, Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);//(Axis2-Axis1)/2;
        Motor4.spin(directionType::fwd, Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);//(Axis2-Axis1)/2;
      
        if(Controller1.ButtonR2.pressing()){
            Motor5.spin(directionType::fwd, 100,pct);
             Motor6.spin(directionType::rev, 100,pct);
        } else if(Controller1.ButtonR1.pressing()){
            Motor5.spin(directionType::rev, 100,pct);
             Motor6.spin(directionType::fwd, 100,pct);
        } else {
            Motor5.spin(directionType::rev,0,pct);
             Motor6.spin(directionType::fwd, 0,pct);
        }
        
        
        if(Controller1.ButtonL2.pressing()){
            Motor7.spin(directionType::fwd, 100,pct);
           
        } else if(Controller1.ButtonL1.pressing()){
            Motor7.spin(directionType::rev, 100,pct);
          
        } else {
            Motor7.spin(directionType::rev,0,pct);
           
        }
        
        
        if(Controller1.ButtonA.pressing()){
          Controller1.rumble(".-.-............");
        }
        if(Controller1.ButtonLeft.pressing()){
          MacroStack();
        }


        
        if(Controller1.ButtonUp.pressing()){
            Motor8.spin(directionType::fwd, 50,pct);
           
        } else if(Controller1.ButtonDown.pressing()){
            Motor8.spin(directionType::rev, 50,pct);
          
        } else {
            Motor8.spin(directionType::rev,0,pct);
           
        }
        Controller1.Screen.clearScreen();
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo 
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to 
    // update your motors, etc.
    // ........................................................................
 
    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
} 