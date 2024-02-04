// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Shooter.h"
#include "Constants.h"

using namespace ShooterConstants;

/*
Todo: will need to update the type of motor used on the intake once it's known
Todo: Will need to make methods that make sense for what the intake will do
- the intake is planned to be a conveyor so will need to turn one direction at a specific speed
- I'm thinking it should be able to be reversed if we need to 
-it'd be nice to know if we have a note loaded 
*/
 Shooter::Shooter() {
  // Implementation of subsystem constructor goes here.
  //frc::PWMSparkMax m_conveyorMotor(kconveyorMotorPort);
  
}

void Shooter::ShootUp() {
    m_ShooterOne.Set(kUpShooterSpeed);
    m_ShooterTwo.Set(kUpShooterSpeed);
    
    frc::SmartDashboard::PutNumber("ShooterStatus",m_ShooterOneEncoder.GetPosition()); 

}

void Shooter::ShootDown() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  
    m_ShooterOne.Set(kDownShooterSpeed);
    m_ShooterTwo.Set(kDownShooterSpeed);
   frc::SmartDashboard::PutNumber("ShooterStatus",m_ShooterOneEncoder.GetPosition()); 
    
}

void Shooter::Stop(){
  frc::SmartDashboard::PutString("ShooterStatus","Idle"); 
  m_ShooterOne.Set(0);
  m_ShooterTwo.Set(0);
  
}


void Shooter::Periodic() {
  // Implementation of subsystem periodic method goes here.

}

void Shooter::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
