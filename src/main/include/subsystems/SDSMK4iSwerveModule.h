#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/DutyCycleEncoder.h> // For pwm absolute encoder

#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/configs/Configs.hpp"

class SDSMK4iSwerveModule {
 public:
  /**
   * Constructs an SDSMK4iSwerveModule and configures the driving and turning
   * motor, encoder, and PID controller. This configuration is specific to the
   * SDS MK4i Module built with NEOs, SPARKS MAX, and an Absolute Encoder.
   */
  
  SDSMK4iSwerveModule(int driveCANId, int turningCANId, int absoluteEncoderCANId,
                    units::angle::radian_t chassisAngularOffset);

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  frc::SwerveModuleState GetState();

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  frc::SwerveModulePosition GetPosition();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  void SetDesiredState(const frc::SwerveModuleState& state);

  /**
   * Zeroes all the SwerveModule encoders.
   */
  void ResetEncoders();

 private:
  rev::CANSparkMax m_drivingSparkMax;
  rev::CANSparkMax m_turningSparkMax;

  rev::SparkRelativeEncoder m_drivingEncoder =
      m_drivingSparkMax.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  ctre::phoenix6::hardware::CANcoder m_turningAbsoluteEncoder;

  rev::SparkPIDController m_drivingPIDController =
      m_drivingSparkMax.GetPIDController();
  rev::SparkPIDController m_turningPIDController =
      m_turningSparkMax.GetPIDController();

  units::angle::radian_t m_chassisAngularOffset{0.0};
  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                        frc::Rotation2d()};
  bool m_turningEncoderInverted = false;
};
