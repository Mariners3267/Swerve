// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/DigitalInput.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Example command factory method.
   */

  void RunArm();
  void ReverseArm();
  void Stop();
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool is_arm_up();
  bool is_arm_down();
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
  rev::CANSparkMax m_armMotor{ArmConstants::kArmCanid, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_ArmEncoder = m_armMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  //rev::CANSparkMax m_conveyorMotor(int deviceID, rev::CANSparkLowLevel::MotorType type);
  frc::DigitalInput m_ampLimitSwitch{ArmConstants::kArmDIOLimitPort};
  frc::DigitalInput m_ampLimitSwitch2{ArmConstants::kArmDIOLimitPortDown};
  // Digital IO Classification
};