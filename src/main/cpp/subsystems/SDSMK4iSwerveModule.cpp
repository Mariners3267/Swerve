#include "subsystems/SDSMK4iSwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <numbers>
#include "Constants.h"

using namespace ModuleConstants;

SDSMK4iSwerveModule::SDSMK4iSwerveModule(const int drivingCANId, const int turningCANId, 
                                         const int absoluteEncoderChannel, 
                                         const double chassisAngularOffset)
    : m_drivingSparkMax(drivingCANId, rev::CANSparkMax::MotorType::kBrushless),
      m_turningSparkMax(turningCANId, rev::CANSparkMax::MotorType::kBrushless),
      m_turningAbsoluteEncoder(absoluteEncoderChannel) {
  // Factory reset for both SPARK MAX controllers
  m_drivingSparkMax.RestoreFactoryDefaults();
  m_turningSparkMax.RestoreFactoryDefaults();

  // Configure driving encoder for SDS-specific units
  m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
  m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

  // Configure absolute encoder for turning
  m_turningAbsoluteEncoder.SetDistancePerRotation(kTurningEncoderPositionFactor);
  m_turningAbsoluteEncoder.SetPositionOffset(0.0); // Adjust this based on calibration
  m_turningAbsoluteEncoder.SetDistancePerRotation(std::numbers::pi * 2); // Full rotation in radians

  // Set up encoder inversion in software
  m_turningEncoderInverted = kTurningEncoderInverted;

  // Set PID gains for the driving motor
  m_drivingPIDController.SetP(kDrivingP);
  m_drivingPIDController.SetI(kDrivingI);
  m_drivingPIDController.SetD(kDrivingD);
  m_drivingPIDController.SetFF(kDrivingFF);
  m_drivingPIDController.SetOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  // Set PID gains for the turning motor
  m_turningPIDController.SetP(kTurningP);
  m_turningPIDController.SetI(kTurningI);
  m_turningPIDController.SetD(kTurningD);
  m_turningPIDController.SetFF(kTurningFF);
  m_turningPIDController.SetOutputRange(kTurningMinOutput, kTurningMaxOutput);

  // Set SPARK MAX configurations
  m_drivingSparkMax.SetIdleMode(kDrivingMotorIdleMode);
  m_turningSparkMax.SetIdleMode(kTurningMotorIdleMode);
  m_drivingSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());
  m_turningSparkMax.SetSmartCurrentLimit(kTurningMotorCurrentLimit.value());

  // Save SPARK MAX settings to flash memory
  m_drivingSparkMax.BurnFlash();
  m_turningSparkMax.BurnFlash();

  // Apply chassis angular offset
  m_chassisAngularOffset = chassisAngularOffset;

  // Initialize desired state
  m_desiredState.angle = frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetDistance()});
  m_drivingEncoder.SetPosition(0);
}

frc::SwerveModuleState SDSMK4iSwerveModule::GetState() const {
  // Apply inversion if needed
  double turningPosition = m_turningAbsoluteEncoder.GetDistance();
  if (m_turningEncoderInverted) {
    turningPosition = -turningPosition;
  }

  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          units::radian_t{turningPosition - m_chassisAngularOffset}};
}

frc::SwerveModulePosition SDSMK4iSwerveModule::GetPosition() const {
  // Apply inversion if needed
  double turningPosition = m_turningAbsoluteEncoder.GetDistance();
  if (m_turningEncoderInverted) {
    turningPosition = -turningPosition;
  }

  return {units::meter_t{m_drivingEncoder.GetPosition()},
          units::radian_t{turningPosition - m_chassisAngularOffset}};
}

void SDSMK4iSwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
  // Apply chassis angular offset to the desired state
  frc::SwerveModuleState correctedDesiredState{};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle =
      desiredState.angle + frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

  // Optimize the reference state to minimize rotation
  frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(
      correctedDesiredState, frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetDistance()}))};

  // Command driving and turning motors
  m_drivingPIDController.SetReference((double)optimizedDesiredState.speed.value(),
                                      rev::CANSparkMax::ControlType::kVelocity);
  m_turningPIDController.SetReference(optimizedDesiredState.angle.Radians().value(),
                                      rev::CANSparkMax::ControlType::kPosition);

  m_desiredState = desiredState;
}

void SDSMK4iSwerveModule::ResetEncoders() { 
  m_drivingEncoder.SetPosition(0); 
}