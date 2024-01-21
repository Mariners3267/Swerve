// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/DigitalInput.h>
#include <frc/motorcontrol/PWMTalonSRX.h>


class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr RunIntake();
  frc2::CommandPtr ReverseIntake();
  frc2::CommandPtr Stop();
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool isLoaded();

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
  frc::DigitalInput m_loadedSensor{IntakeConstants::kIntakeLimitSwitchDIOPort};
  //rev::CANSparkMax m_conveyorMotor{IntakeConstants::kConveyorCanId, rev::CANSparkLowLevel::MotorType::kBrushless};
  frc::PWMTalonSRX m_conveyorMotor{IntakeConstants::kConveyorPWMPort};
  //rev::CANSparkMax m_conveyorMotor(int deviceID, rev::CANSparkLowLevel::MotorType type);
};
