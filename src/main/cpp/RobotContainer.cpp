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
#include <frc2/command/WaitCommand.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc2/command/FunctionalCommand.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Arm.h"
#include <photon/PhotonUtils.h>
#include "utils/AprilTagData.h"
#include <frc/DriverStation.h>
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  aprilTag.addAprilTagData(1, 122_cm, "source", "blue"); //on the right side of source
  aprilTag.addAprilTagData(2, 122_cm, "source", "blue"); //on the left side of source
  aprilTag.addAprilTagData(3, 132_cm, "speaker", "red"); //shifted 43_cm toward drivers station on speaker
  aprilTag.addAprilTagData(4, 132_cm, "speaker", "red"); //vertically centered on speaker
  aprilTag.addAprilTagData(5, 122_cm, "amp", "red");     //vertically centered on amp
  aprilTag.addAprilTagData(6, 122_cm, "amp", "blue");    //vertically centered on amp
  aprilTag.addAprilTagData(7, 132_cm, "speaker", "blue"); //vertically centered on speaker
  aprilTag.addAprilTagData(8, 132_cm, "speaker", "blue"); //shifted 43_cm toward drivers station on speaker
  aprilTag.addAprilTagData(9, 122_cm, "source", "red"); //on the right side of source
  aprilTag.addAprilTagData(10, 122_cm, "source", "red"); //on the left side of source
  aprilTag.addAprilTagData(11, 121_cm, "stage", "red");
  aprilTag.addAprilTagData(12, 121_cm, "stage", "red");
  aprilTag.addAprilTagData(13, 121_cm, "stage", "red");
  aprilTag.addAprilTagData(14, 121_cm, "stage", "blue");
  aprilTag.addAprilTagData(15, 121_cm, "stage", "blue");
  aprilTag.addAprilTagData(16, 121_cm, "stage", "blue");




  // Configure the button bindings
  ConfigureButtonBindings();

  m_chooser.SetDefaultOption(kAutoNameCenter, kAutoNameCenter);
  m_chooser.AddOption(kAutoNameRed2, kAutoNameRed2);
  m_chooser.AddOption(kAutoNameRed3, kAutoNameRed3);
  m_chooser.AddOption(kAutoNameRedLongRun, kAutoNameRedLongRun);
  m_chooser.AddOption(kAutoNameBlueLongRun, kAutoNameBlueLongRun);
  m_chooser.AddOption(kAutoNameBlue2, kAutoNameBlue2);
  m_chooser.AddOption(kAutoNameBlue3, kAutoNameBlue3);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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
            false, true);
      },
      {&m_drive}));
     /* m_lifter.SetDefaultCommand(frc2::RunCommand(
       [this] {
         m_lifter.RunLifters(m_driverController.GetLeftTriggerAxis(), m_driverController.GetRightTriggerAxis());
       },
      {&m_lifter})); */
   m_lifter.SetDefaultCommand(frc2::RunCommand(
       [this] {
         m_lifter.RunLifters(m_coDriverController.GetLeftY(), m_coDriverController.GetRightY());
       },
      {&m_lifter}));

}



void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kStart)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.ZeroHeading(); }, {&m_drive}));
  
  
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
   

    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kLeftBumper)
      .OnTrue(new frc2::InstantCommand([this] { m_camera.CameraToggle(); }, {&m_camera}));
//GetLeftStickButtonPressed    
          
    frc2::JoystickButton(&m_driverController,
                    frc::XboxController::Button::kX)
    .OnTrue(new frc2::RunCommand([this] { 
        m_arm.ReverseArm(); 
    }, {&m_arm}))
    .OnFalse(new frc2::RunCommand([this] { 
        m_arm.Stop(); 
    }, {&m_arm}));

    frc2::JoystickButton(&m_driverController,
                    frc::XboxController::Button::kB)
    .OnTrue(new frc2::RunCommand([this] { 
        m_arm.RunArm(); 
    }, {&m_arm}))
    .OnFalse(new frc2::RunCommand([this] { 
        m_arm.Stop(); 
    }, {&m_arm}));

//Second Controller
      frc2::JoystickButton(&m_coDriverController,
                       frc::XboxController::Button::kRightBumper)
      .OnTrue(new frc2::RunCommand([this] { 
        m_shooter.mrBloadintoshooter();
         }, {&m_shooter}))
     .OnFalse(new frc2::RunCommand([this] { 
        m_shooter.Stop(); 
    }, {&m_shooter}));

          frc2::JoystickButton(&m_coDriverController,
                       frc::XboxController::Button::kRightBumper)
      .OnTrue(new frc2::RunCommand([this] { 
        m_intake.RunIntake();
         }, {&m_intake}))
     .OnFalse(new frc2::RunCommand([this] { 
        m_intake.Stop(); 
    }, {&m_intake}));

  frc2::JoystickButton(&m_coDriverController,
                       frc::XboxController::Button::kA)
      .OnTrue(new frc2::RunCommand([this] { 
        m_shooter.ShootUp();
         }, {&m_shooter}))
     .OnFalse(new frc2::RunCommand([this] { 
        m_shooter.Stop(); 
    }, {&m_shooter}));
    frc2::JoystickButton(&m_coDriverController,
                       frc::XboxController::Button::kB)
      .OnTrue(new frc2::RunCommand([this] { 
        m_shooter.ShootDown();
         }, {&m_shooter}))
     .OnFalse(new frc2::RunCommand([this] { 
        m_shooter.Stop(); 
    }, {&m_shooter}));

    frc2::JoystickButton(&m_coDriverController,
                       frc::XboxController::Button::kX)
      .OnTrue(new frc2::RunCommand([this] { m_intake.RunIntake(); }, {&m_intake}))
      .OnFalse(new frc2::RunCommand([this] { 
        m_intake.Stop(); 
    }, {&m_intake}));     

    frc2::JoystickButton(&m_coDriverController, 
                       frc::XboxController::Button::kY)
      .OnTrue(new frc2::RunCommand([this] { m_intake.ReverseIntake(); }, {&m_intake}))
      .OnFalse(new frc2::RunCommand([this] { 
        m_intake.Stop(); 
    }, {&m_intake}));  
    

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
    frc::Pose2d{0_ft, 0_ft, 0_deg},

        // positive 2nd number = left, negative = right
        // Pass through these two interior waypoints, making an 's' curve path
        {
        frc::Translation2d{1_ft, 0_ft},
        },         
      // End 1 meters straight ahead of where we started, facing forward
      frc::Pose2d{5_ft, 0_ft, 0_deg},
      // Pass the config
      config); 

       // An example trajectory to follow.  All units in meters.
  auto rotateExampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{2_m, 0_m, 0_deg},

        // positive 2nd number = left, negative = right
        // Pass through these two interior waypoints, making an 's' curve path
        {
        frc::Translation2d{2.1_m, 0_m},
        frc::Translation2d{2.2_m, 0_m},
        },         
      // End 1 meters straight ahead of where we started, facing forward
      frc::Pose2d{2.3_m, 0_m, 90_deg},
      // Pass the config
      config);
       auto backwardExampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{5_ft, 0_m, 0_deg},

        // positive 2nd number = left, negative = right
        // Pass through these two interior waypoints, making an 's' curve path
        {
        frc::Translation2d{3_ft, 0_m},
        },         
      // End 1 meters straight ahead of where we started, facing forward
      frc::Pose2d{0_ft, 0_m, 0_deg},
      // Pass the config
      config);

       auto Blue2Trajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_in, 0_in, 0_deg},

        // positive 2nd number = left, negative = right
        {
        frc::Translation2d{22_in, 0_in}
        },         
      frc::Pose2d{45_in, 0_in, 0_deg},
      // Pass the config
      config);
       auto Blue2Trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{45_in, 0_in, 0_deg},

        // positive 2nd number = left, negative = right
        {
        frc::Translation2d{60_in, 0_in},
        },         
      frc::Pose2d{81_in, 0_in, 0_deg},
      // Pass the config
      config);
       auto Blue2Trajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{44_in, 0_in, 0_deg},

        // positive 2nd number = left, negative = right
        {
        frc::Translation2d{70_in, 0_in},
        },         
      frc::Pose2d{76_in, 0_in, 0_deg},
      // Pass the config
      config);
       auto Blue2Trajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{76_in, 0_in, 0_deg},

        // positive 2nd number = left, negative = right
        {
        frc::Translation2d{64_in, 0_in},
        },         
      frc::Pose2d{52_in, 0_in, 0_deg},
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


    frc2::SwerveControllerCommand<4> rotateControllerCommand(
      rotateExampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

          frc2::SwerveControllerCommand<4> backwardControllerCommand(
      backwardExampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

    frc2::SwerveControllerCommand<4> Blue2SwerveCommand1(
      Blue2Trajectory1, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});
    frc2::SwerveControllerCommand<4> Blue2SwerveCommand2(
      Blue2Trajectory2, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

    frc2::SwerveControllerCommand<4> Blue2SwerveCommand3(
      Blue2Trajectory3, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});
    frc2::SwerveControllerCommand<4> Blue2SwerveCommand4(
      Blue2Trajectory4, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});
  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());





  // no auto
    
  
   if (m_chooser.GetSelected() == "Red Speaker Amp") {
          frc::SmartDashboard::PutString("Auto Running", "Red Speaker Amp");
    return new frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] { 
        m_drive.ZeroHeading();    
      }, {&m_drive}),
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
     // std::move(Blue2SwerveCommand1),
      frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
        frc2::FunctionalCommand(
        // Reset gyro
        [this] {
            m_drive.SetPreviousDegrees();
            m_drive.ZeroHeading();},
        //Start rotating at start of command
       // [this] {m_drive.Drive(.1_mps, 0_mps, -0.125_rad_per_s, false, false); },
        [this] {m_drive.Drive(.2_mps, 0_mps, 0.095_rad_per_s, false, false); },
        //Stop rotating at end of command
        [this] (bool interrupted) {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
        //End the comand when the robot's rotation exceeds 40 degrees
        [this] {
            //break out of turn if we're not getting any pigeon readings
            //if(m_drive.GetDegrees() == m_drive.GetPreviousDegrees()){
            m_drive.SetPreviousDegrees();
            frc::SmartDashboard::PutNumber("previousdegrees", m_drive.GetPreviousDegrees());
            frc::SmartDashboard::PutNumber("currentdegrees", m_drive.GetDegrees());
            return abs(m_drive.GetDegrees()) >= 52;
            // overshoots ~8 degrees 
            
            },
        {&m_drive}
        ),
      //std::move(Blue2SwerveCommand2),
     frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
          frc2::FunctionalCommand(
        // Reset gyro
        [this] {
            m_drive.SetPreviousDegrees();
            m_drive.ZeroHeading();},
        //Start rotating at start of command
       // [this] {m_drive.Drive(.1_mps, 0_mps, -0.125_rad_per_s, false, false); },
        [this] {m_drive.Drive(-0.125_mps, 0_mps, 0.155_rad_per_s, false, false); },
        //Stop rotating at end of command
        [this] (bool interrupted) {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
        //End the comand when the robot's rotation exceeds 40 degrees
        [this] {
            //break out of turn if we're not getting any pigeon readings
            //if(m_drive.GetDegrees() == m_drive.GetPreviousDegrees()){
            m_drive.SetPreviousDegrees();
            frc::SmartDashboard::PutNumber("previousdegrees", m_drive.GetPreviousDegrees());
            frc::SmartDashboard::PutNumber("currentdegrees", m_drive.GetDegrees());
            return abs(m_drive.GetDegrees()) >= 78;
            // overshoots ~8 degrees 
            
            },
        {&m_drive}
        ),
       frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.ZeroHeading();}, {&m_drive}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.05_mps, 0.11_mps, 0_rad_per_s, false, false);}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0.0_mps, 0_rad_per_s, false, false);}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.mrBloadintoshooter();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_arm.RunArm();}, {&m_arm}),
      frc2::WaitCommand(0.66_s),
      frc2::InstantCommand([this] {m_arm.Stop();}, {&m_arm}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.mrBloadintoshooter();}, {&m_shooter}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_arm.ReverseArm();}, {&m_arm}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_arm.Stop();}, {&m_arm})
  ); 
  
  }
     else if (m_chooser.GetSelected() == "Blue 3 notes") {
            frc::SmartDashboard::PutString("Auto Running", "Blue 3 notes");
    return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_drive.Drive(0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.5_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.4_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.3_s),
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0.3_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.56_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(0.25_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.7_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.6_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.25_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.7_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, -0.3_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.63_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_shooter.ShootUp();}, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake})
      //frc2::InstantCommand([this] {m_getridofbritishpeople.getridofbritishpeople(); } {&m_getridofbritishpeople}), //this is to get rid of british people

  ); 
  }
  else if (m_chooser.GetSelected() == "Blue Long Run") {
        frc::SmartDashboard::PutString("Auto Running", "Blue Long Run");
    return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_drive.Drive(0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.5_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.4_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.3_s),
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake})
  ); 
  
  }  
else if (m_chooser.GetSelected() == "Center") {
        frc::SmartDashboard::PutString("Auto Running", "Center");
    return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_drive.Drive(0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.5_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.4_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.3_s),
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake})
  ); 
  
  }  
  else if (m_chooser.GetSelected() == "Red Long Run") {
        frc::SmartDashboard::PutString("Auto Running", "Red Long Run");
    return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(2_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0.1_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive})

  ); 
  
  }  
  else if (m_chooser.GetSelected() == "Blue Speaker Amp") {
      m_drive.ResetOdometry(Blue2Trajectory1.InitialPose());
      m_drive.ResetOdometry(Blue2Trajectory2.InitialPose());
      m_drive.ResetOdometry(Blue2Trajectory3.InitialPose());  
            frc::SmartDashboard::PutString("Auto Running", "Blue Speaker Amp");       
    return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { 
        m_drive.ZeroHeading();    
      }, {&m_drive}),
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
     // std::move(Blue2SwerveCommand1),
      frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
        frc2::FunctionalCommand(
        // Reset gyro
        [this] {
            m_drive.SetPreviousDegrees();
            m_drive.ZeroHeading();},
        //Start rotating at start of command
       // [this] {m_drive.Drive(.1_mps, 0_mps, -0.125_rad_per_s, false, false); },
        [this] {m_drive.Drive(.175_mps, 0_mps, -0.06_rad_per_s, false, false); },
        //Stop rotating at end of command
        [this] (bool interrupted) {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
        //End the comand when the robot's rotation exceeds 40 degrees
        [this] {
            //break out of turn if we're not getting any pigeon readings
            //if(m_drive.GetDegrees() == m_drive.GetPreviousDegrees()){
            m_drive.SetPreviousDegrees();
            frc::SmartDashboard::PutNumber("previousdegrees", m_drive.GetPreviousDegrees());
            frc::SmartDashboard::PutNumber("currentdegrees", m_drive.GetDegrees());
            return abs(m_drive.GetDegrees()) >= 37;
            // overshoots ~8 degrees 
            
            },
        {&m_drive}
        ),
      //std::move(Blue2SwerveCommand2),
      frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
              frc2::FunctionalCommand(
        // Reset gyro
        [this] {
            m_drive.SetPreviousDegrees();
            m_drive.ZeroHeading();},
        //Start rotating at start of command
       // [this] {m_drive.Drive(.1_mps, 0_mps, -0.125_rad_per_s, false, false); },
        [this] {m_drive.Drive(-0.125_mps, 0_mps, -0.15_rad_per_s, false, false); },
        //Stop rotating at end of command
        [this] (bool interrupted) {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
        //End the comand when the robot's rotation exceeds 40 degrees
        [this] {
            //break out of turn if we're not getting any pigeon readings
            //if(m_drive.GetDegrees() == m_drive.GetPreviousDegrees()){
            m_drive.SetPreviousDegrees();
            frc::SmartDashboard::PutNumber("previousdegrees", m_drive.GetPreviousDegrees());
            frc::SmartDashboard::PutNumber("currentdegrees", m_drive.GetDegrees());
            return abs(m_drive.GetDegrees()) >= 80;
            // overshoots ~8 degrees 
            
            },
        {&m_drive}
        ),
        frc2::WaitCommand(0.1_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.ZeroHeading();}, {&m_drive}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.1_mps, -0.08_mps, 0_rad_per_s, false, false);}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0.0_mps, 0_rad_per_s, false, false);}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.mrBloadintoshooter();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_arm.RunArm();}, {&m_arm}),
      frc2::WaitCommand(0.66_s),
      frc2::InstantCommand([this] {m_arm.Stop();}, {&m_arm}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.mrBloadintoshooter();}, {&m_shooter}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::WaitCommand(0.2_s),
      frc2::InstantCommand([this] {m_arm.ReverseArm();}, {&m_arm}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_arm.Stop();}, {&m_arm})
     /* std::move(backwardControllerCommand),
      frc2::WaitCommand(1_s),
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}) */
  ); }
    else if (m_chooser.GetSelected() == "Red 3 notes") {
      frc::SmartDashboard::PutString("Auto Running", "Red 3 notes");
    return new frc2::SequentialCommandGroup(
  frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s), //this waits 9 seconds A
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s), //this waits 12 seconds
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_drive.Drive(0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.5_s), // this waits 45 seconds
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::WaitCommand(0.1_s), //this waits 6 seconds
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::WaitCommand(0.1_s), //this waits 0.1 seconds
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.35_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.4_s), //this waits 1.4 seconds
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.3_s), //this waits 0.3 seconds
      frc2::InstantCommand([this] { m_shooter.ShootUp(); }, {&m_shooter}),
      frc2::WaitCommand(0.75_s), //this waits 0.75 seconds
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s), //this waits 0.5 seconds
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, -0.3_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.52_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(0.25_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.7_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.6_s),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_drive.Drive(-0.25_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(1.8_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_intake.ReverseIntake();}, {&m_intake}),
      frc2::InstantCommand([this] {m_shooter.ShootDown();}, {&m_shooter}),
      frc2::WaitCommand(0.2_s),
      //A
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake}),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0.3_rad_per_s, false, false);}, {&m_drive}),
      frc2::WaitCommand(0.6_s),
      frc2::InstantCommand([this] {m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);}, {&m_drive}),
      frc2::InstantCommand([this] {m_shooter.ShootUp();}, {&m_shooter}),
      frc2::WaitCommand(0.75_s),
      frc2::InstantCommand([this] {m_intake.RunIntake();}, {&m_intake}),
      frc2::WaitCommand(0.5_s),
      frc2::InstantCommand([this] {m_shooter.Stop();}, {&m_shooter}),
      frc2::InstantCommand([this] {m_intake.Stop();}, {&m_intake})
  ); 
  
  }  
  else{
    return new frc2::SequentialCommandGroup(

     frc2::InstantCommand([this] { 
            frc::SmartDashboard::PutString("Auto Running", "Nope");
frc::SmartDashboard::PutString("Auto Routine Selected", m_chooser.GetSelected());

      }, {&m_drive})
     
  );
  }
  
    
}
