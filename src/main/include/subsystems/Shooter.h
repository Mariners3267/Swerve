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
  frc2::CommandPtr SpinShooter();
  frc2::CommandPtr StopShooter();
  frc2::CommandPtr SpinFeeder();
  frc2::CommandPtr StopFeeder();
  
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool isNoteLoaded();
  bool isShooterUpToSpeed();

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
  frc::DigitalInput m_noteLoadedSensor{ShooterConstants::kNoteLoadedDigitalInputPort};
  rev::CANSparkMax m_shooterMotor{ShooterConstants::kShooterCanId, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_shooterEncoder = m_shooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::CANSparkMax m_feederMotor{ShooterConstants::kFeederCanId, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_feederEncoder = m_feederMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

};
