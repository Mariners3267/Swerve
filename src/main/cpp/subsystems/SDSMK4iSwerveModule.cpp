#include "subsystems/SDSMK4iSwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <numbers>
#include "Constants.h"

using namespace ModuleConstants;
 
SDSMK4iSwerveModule::SDSMK4iSwerveModule(const int drivingCANId, const int turningCANId, 
                                         const int absoluteEncoderCANId, 
                                         const units::angle::radian_t chassisAngularOffset)
    : m_drivingSparkMax(drivingCANId, rev::CANSparkMax::MotorType::kBrushless),
      m_turningSparkMax(turningCANId, rev::CANSparkMax::MotorType::kBrushless),
      m_turningAbsoluteEncoder(absoluteEncoderCANId) {
  // Factory reset for both SPARK MAX controllers
  m_drivingSparkMax.RestoreFactoryDefaults();
  m_turningSparkMax.RestoreFactoryDefaults();

  // Configure driving encoder for SDS-specific units
  m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
  m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

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
  m_desiredState.angle = frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition().GetValue()});
    m_drivingEncoder.SetPosition(0);
}

frc::SwerveModuleState SDSMK4iSwerveModule::GetState(){
  // Get the absolute position in turns
  units::angle::turn_t rawPosition = m_turningAbsoluteEncoder.GetPosition().GetValue();
  
  // Apply inversion if needed
  units::angle::turn_t adjustedPosition = m_turningEncoderInverted ? -rawPosition : rawPosition;

  // Convert turns to radians
  units::angle::radian_t angleInRadians = adjustedPosition * 2.0 * M_PI;

  // Ensure m_chassisAngularOffset is in radians
  units::angle::radian_t chassisOffsetInRadians = units::angle::radian_t{m_chassisAngularOffset};  // If m_chassisAngularOffset is already in radians, this is redundant but safe

  // Apply the offset
  angleInRadians = angleInRadians - chassisOffsetInRadians;

  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          frc::Rotation2d(angleInRadians)};
}

frc::SwerveModulePosition SDSMK4iSwerveModule::GetPosition() {
    // Get the absolute position in turns from the turning encoder
    units::angle::turn_t rawPosition = m_turningAbsoluteEncoder.GetPosition().GetValue();

    // Apply inversion if needed
    units::angle::turn_t adjustedPosition = m_turningEncoderInverted ? -rawPosition : rawPosition;

    // Convert turns to radians (turns × 2π = radians)
    units::angle::radian_t angleInRadians = adjustedPosition * 2.0 * M_PI;

    // Apply the chassis angular offset - ensure it's a radian_t
    angleInRadians -= m_chassisAngularOffset; // Both should now be in radians

    // Get the driving encoder position (assuming it returns meters)
    units::meter_t distance = units::meter_t{m_drivingEncoder.GetPosition()};

    // Return the swerve module position
    return {distance, frc::Rotation2d(angleInRadians)};
}

void SDSMK4iSwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
  // Apply chassis angular offset to the desired state
  frc::SwerveModuleState correctedDesiredState{};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle =
      desiredState.angle + frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

  // Optimize the reference state to minimize rotation
  frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(
      correctedDesiredState, frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition().GetValue()}))};

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