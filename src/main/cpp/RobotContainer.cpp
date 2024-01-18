// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Arm.h"
#include <photon/PhotonUtils.h>

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));


    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kX)
      .WhileTrue(new frc2::RunCommand([this] { m_intake.RunIntake(); }, {&m_intake}));     

    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kY)
      .WhileTrue(new frc2::RunCommand([this] { m_intake.ReverseIntake(); }, {&m_intake}));  

    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kLeftBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_intake.Stop(); }, {&m_intake}));      


    frc2::JoystickButton(&m_driverController,
                    frc::XboxController::Button::kA)
    .OnTrue(new frc2::RunCommand([this] { 
        m_arm.RunArm(); 
    }, {&m_arm}))
    .OnFalse(new frc2::RunCommand([this] { 
        m_arm.Stop(); 
    }, {&m_arm}));

    frc2::JoystickButton(&m_driverController,
                    frc::XboxController::Button::kB)
    .WhileTrue(new frc2::RunCommand([this] { 
          m_arm.Stop();
        }, {&m_arm})); 


//Second Controller
    frc2::JoystickButton(&m_coDriverController,
                       frc::XboxController::Button::kLeftBumper)
      .WhileTrue(new frc2::RunCommand([this] { 
        //get the camera target info and pass as params to 
        
        photon::PhotonPipelineResult result = camera.GetLatestResult();
        if (result.HasTargets()) {
            double Yehaw = result.GetBestTarget().GetYaw();
            int targetID = result.GetBestTarget().GetFiducialId();

            //TODO:
            //Different targets are at different heights
            //need to create a hash or map to pass in the target height for this targetID

        
/*
SOURCE AprilTags (IDs 1, 2, 9, and 10) are mounted to the SOURCE wall. The bottom of each tag’s panel is 4 ft.
⅛ in. (~122 cm) above the carpet and 1 ft. 7⅜ in. (~50 cm) from the vertical center of the SOURCE.

SPEAKER AprilTags (IDs 3, 4, 7, and 8) are mounted to the ALLIANCE WALL. The bottom of each tag’s panel is
4 ft. 3⅞ in. (~132 cm) above the carpet. 1 tag (IDs 4 and 7) is vertically centered above each SUBWOOFER. The
2nd tag (IDs 3 and 8) is shifted toward DRIVER STATION 2 and the edge of its panel is 1 ft. 5 in. (~43 cm) from
the vertical center of the SPEAKER ALLIANCE WALL plastic. 

AMP AprilTag panels (IDs 5 and 6) are 4 ft. ⅛ in. (~122 cm) above the carpet and centered vertically above the
AMP wall. 

STAGE AprilTag plates (IDs 11-16) are 3 ft. 11½ in. (~121 cm) above the carpet and centered vertically on each
of the 3 wide faces of the STAGE core. These tags are behind ¼-in. (~6 mm) thick polycarbonate. 
*/
            if (targetID == 1) {

                units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
                units::degree_t{result.GetBestTarget().GetPitch()});
                m_drive.PhotonDrive(targetID, Yehaw, range); 
            }
            
            else {
            //If we don't care about this target - leave it in controll of the driver
            m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true);
            }
        }
        else {
            //If we don't have any vision targets - leave it in controll of the driver
            m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true);
        }
      }, {&m_drive}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand([this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },{}),
      frc2::RunCommand([this] { 
        m_arm.RunArm(); 
    }, {&m_arm})
        
  );
}