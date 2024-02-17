// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

class Lifter : public frc2::SubsystemBase {
 public:
  Lifter();

  /**
   * Example command factory method.
   */

  void RunLifters(double leftLifterSpeed, double rightLifterSpeed);
  void LiftersUp();
  void Stop();
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  double GetLeftLifterPosition();
  double GetRightLifterPosition();
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
  rev::CANSparkMax m_leftLifterMotor{LifterConstants::kLeftLifterCanid, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_leftLifterEncoder = m_leftLifterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::CANSparkMax m_rightLifterMotor{LifterConstants::kRightLifterCanid, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_rightLifterEncoder = m_rightLifterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

};