// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Camera.h"
#include "Constants.h"

using namespace CameraConstants;

/*
Todo: will need to update the type of motor used on the intake once it's known
Todo: Will need to make methods that make sense for what the intake will do
- the intake is planned to be a conveyor so will need to turn one direction at a specific speed
- I'm thinking it should be able to be reversed if we need to 
-it'd be nice to know if we have a note loaded 
*/
Camera::Camera() {
  // Implementation of subsystem constructor goes here.
  //frc::PWMSparkMax m_conveyorMotor(kconveyorMotorPort);
}

void Camera::CameraToggle() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
   if(cameraServo.GetPosition()>kCameraDownPOS) {
    cameraServo.SetAngle(kCameraDownPOS);
    
   } else {
    cameraServo.SetAngle(kCameraUpPOS);
   }

 // m_armMotor.Set(kArmSpeed);
  // frc::SmartDashboard::PutNumber("Arm_Angle",m_ArmEncoder.GetPosition()); 
   
  //RunOnce creates a command that calls a lambda once, and then finishes.
}


void Camera::Periodic() {
  // Implementation of subsystem periodic method goes here.

}

void Camera::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}



