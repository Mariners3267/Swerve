// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/DigitalInput.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  /**
   * Example command factory method.
   */

  void ShootUp();
  void ShootDown();
  void Stop();
  


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_ShooterTwo{ShooterConstants::kShooterMotorTwoCanId, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_ShooterOne{ShooterConstants::kShooterMotorOneCanId, rev::CANSparkLowLevel::MotorType::kBrushless}; 
  rev::SparkRelativeEncoder m_ShooterOneEncoder = m_ShooterOne.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  //rev::CANSparkMax m_conveyorMotor(int deviceID, rev::CANSparkLowLevel::MotorType type);
};
