// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Intake.h"
#include "subsystems/Arm.h"
#include "subsystems/Lifter.h"
#include <photon/PhotonCamera.h>
#include "utils/AprilTagData.h"
#include "subsystems/Shooter.h"
#include "subsystems/Camera.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_coDriverController{OIConstants::kCoDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  Intake m_intake;
  Arm m_arm; 
  AprilTagData aprilTag;
  Shooter m_shooter;
  Lifter m_lifter;
  Camera m_camera;
  photon::PhotonCamera camera{"photonvision"};

  // The chooser for the autonomous routines
  //frc::SendableChooser<frc2::Command*> m_chooser;
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameCenter = "Center";
  const std::string kAutoNameRed2 = "Red Speaker Amp";
  const std::string kAutoNameRed3 = "Red 3 notes";
  const std::string kAutoNameBlue2 = "Blue Speaker Amp";
  const std::string kAutoNameBlue3 = "Blue 3 notes";
  const std::string kAutoNameRedLongRun = "Red Long Run";
  const std::string kAutoNameBlueLongRun = "Blue Long Run";
  std::string m_autoSelected;

  void ConfigureButtonBindings();
};