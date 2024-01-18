// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

class Pivot : public frc2::SubsystemBase {
 public:
  Pivot();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr RunPivot();
  frc2::CommandPtr ReversePivot();
  frc2::CommandPtr Stop();
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool is_arm_up();

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
  rev::CANSparkMax m_Pivot{PivotConstants::kPivotCanid, rev::CANSparkLowLevel::MotorType::kBrushless};
  //rev::CANSparkMax m_conveyorMotor(int deviceID, rev::CANSparkLowLevel::MotorType type);
};