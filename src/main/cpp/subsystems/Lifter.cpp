// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Lifter.h"
#include "Constants.h"

using namespace LifterConstants;
//A
/*
Todo: will need to update the type of motor used on the intake once it's known
Todo: Will need to make methods that make sense for what the intake will do
- the intake is planned to be a conveyor so will need to turn one direction at a specific speed
- I'm thinking it should be able to be reversed if we need to 
-it'd be nice to know if we have a note loaded 
*/
Lifter::Lifter() {
  // Implementation of subsystem constructor goes here.
  //frc::PWMSparkMax m_conveyorMotor(kconveyorMotorPort);
 // m_armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
 m_rightLifterMotor.SetInverted(true);
}

void Lifter::RunLifters(double leftLifterSpeed, double rightLifterSpeed) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.

if (leftLifterSpeed > 0.99){
  leftLifterSpeed = 0.99;
}
else if (leftLifterSpeed < -0.99){
  leftLifterSpeed = -0.99;
}
else {
  leftLifterSpeed = 0;
}
//A
if (rightLifterSpeed > 0.99){
  rightLifterSpeed = 0.99;
}
else if (rightLifterSpeed < -0.99){
  rightLifterSpeed = -0.99;
}
else {
  rightLifterSpeed = 0;
}


   m_leftLifterMotor.Set(leftLifterSpeed);
   m_rightLifterMotor.Set(rightLifterSpeed);
}

\

void Lifter::Stop(){
  m_leftLifterMotor.Set(0);
  m_rightLifterMotor.Set(0);
}

void Lifter::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("LeftLifter",m_leftLifterEncoder.GetPosition()); 
  frc::SmartDashboard::PutNumber("RightLifter switch",m_rightLifterEncoder.GetPosition()); 
}

void Lifter::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
} //A