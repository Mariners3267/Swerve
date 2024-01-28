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
frc2::CommandPtr Shooter::SpinShooter() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
     return RunOnce([ this ] { 
      m_shooterMotor.Set(ShooterConstants::kShooterSpeed);
      });
  //RunOnce creates a command that calls a lambda once, and then finishes.
}

frc2::CommandPtr Shooter::StopShooter() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([ this ] { 
      m_shooterMotor.Set(0);
      });
}

frc2::CommandPtr Shooter::SpinFeeder() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
     return RunOnce([ this ] { 
      m_feederMotor.Set(ShooterConstants::kFeederSpeed);
      });
  //RunOnce creates a command that calls a lambda once, and then finishes.
}

frc2::CommandPtr Shooter::StopFeeder() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([ this ] { 
      m_feederMotor.Set(0);
      });
}


bool Shooter::isNoteLoaded() {
  // Query some boolean state, such as a digital limit switch
  //The shooter will also be able to query this state of this limit switch 
  //If we are loaded, stop trying to load.
  return m_noteLoadedSensor.Get();
  //return false;
}

bool Shooter::isShooterUpToSpeed() {
  // Query some boolean state, such as a digital limit switch
  //The shooter will also be able to query this state of this limit switch 
  //If we are loaded, stop trying to load.
  return m_shooterEncoder.GetVelocity() > ShooterConstants::kShooterRPMCheck ? true : false;
  //return false;
}

void Shooter::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void Shooter::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
