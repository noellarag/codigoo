// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.AxisArmCmd;
import frc.robot.commands.EmergencyStopCmd;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.ArmCmd;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Axis;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final XboxController m_xboxController = new XboxController(0);
  private final Axis m_axis = new Axis();

    
  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    chooser.addOption("Straight and Docked", loadTrajectoryToRamseteCommand(
        "/home/lvuser/deploy/pathplanner/generatedJSON/FrontAndDocked.wpilib.json",
        true));
    chooser.addOption("Side: Reverse and Out", loadTrajectoryToRamseteCommand("/home/lvuser/deploy/pathplanner/generatedJSON/ReverseAndOut.wpilib.json", true));
    chooser.addOption("1 meter straight", loadTrajectoryToRamseteCommand("/home/lvuser/deploy/pathplanner/generatedJSON/Straight1meter.wpilib.json", true));

    Shuffleboard.getTab("Autonomous").add(chooser);
    
  }

  public Command loadTrajectoryToRamseteCommand(String filename, boolean resetOdometry) {
    m_drivetrain.resetOdometry(new Pose2d());
    m_drivetrain.setSMARTLimit();
    Trajectory trajectory;

  
    try {
      
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch(IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to open trajectory" + filename);
      return new InstantCommand();
    }
    
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose, new RamseteController(
      DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamsetZeta), new SimpleMotorFeedforward(DrivetrainConstants.KsVolts, 
      DrivetrainConstants.ksVoltSecondsPerMeter, DrivetrainConstants.ksVoltSecondsSquaredperMeter), 
      DrivetrainConstants.kDriveKinematics, m_drivetrain::getWheelSpeeds, new PIDController(DrivetrainConstants.kPDriveVel, 0 , 0)
      , new PIDController(DrivetrainConstants.kPDriveVel, 0, 0), m_drivetrain::tankDriveVolts, m_drivetrain);

    if (resetOdometry) {
      return new SequentialCommandGroup(new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } 
    else {
      return ramseteCommand;
    }
    
    
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(chooser.getSelected(), new BalanceCmd(m_drivetrain));
  }

  public Command getTeleopCommand() {
    return new ParallelCommandGroup(new ArcadeDriveCmd(m_drivetrain, m_xboxController), 
   new ArmCmd(m_armSubsystem, m_xboxController), new AxisArmCmd(m_axis, m_xboxController));
  }
}