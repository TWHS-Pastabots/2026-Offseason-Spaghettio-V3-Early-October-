// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final double demoSpeed = 0.25;

  // The driver's and Operator's controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Register Named Commands
    NamedCommands.registerCommand("Intake", intakeCommand());
    new EventTrigger("run intake").whileTrue(Commands.print("running intake"));
    NamedCommands.registerCommand("Shoot", shootCommand());
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * demoSpeed, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * demoSpeed, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * demoSpeed, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    
    m_intake.setDefaultCommand(new RunCommand(()-> m_intake.setIntake(0), m_intake));
    m_shooter.setDefaultCommand(new RunCommand(()-> m_shooter.setShooter(0), m_shooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    /**
     * Driver Controls
     */

    m_driverController.x().onTrue(new RunCommand(()->m_robotDrive.setX()));
    m_driverController.start().onTrue(new RunCommand(()-> m_robotDrive.zeroHeading()));
    

    /**
     * "Operator" Controls
     */

    // Starts Handoff when left trigger is pressed
    m_driverController.leftTrigger().whileTrue(new RunCommand(() -> {
      m_shooter.setSetpoint(8.5);
      m_intake.intake();
      m_shooter.feedIn();
    }, m_shooter, m_intake));

    // Intake Controls
    m_driverController.leftBumper().whileTrue(new RunCommand(() -> m_intake.reverseIntake(), m_intake));
    m_driverController.a().onTrue(new RunCommand(() -> m_intake.stopIntake(), m_intake));
    
    //Shooter Controls
    m_driverController.rightTrigger().whileTrue(new RunCommand(() -> {
      m_shooter.feedIn();
      m_shooter.launch();
    }, m_shooter));
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_shooter.reverseLaunch(), m_shooter));
    m_driverController.povLeft().whileTrue(new RunCommand(() -> m_shooter.feedIn(), m_shooter));
    m_driverController.povRight().whileTrue(new RunCommand(() -> m_shooter.feedOut(), m_shooter));
    m_driverController.b().onTrue(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));
    m_driverController.y().onTrue(m_shooter.flip());
  }


  public Command intakeCommand() {
    return Commands.run(() -> leftTriggerCopy(), m_intake, m_shooter);
  }


  public Command shootCommand() {
    return Commands.run(() -> rightTriggerCopy(), m_shooter);
  }


  public void leftTriggerCopy() {
    m_shooter.setSetpoint(8.5);
    m_intake.intake();
    m_shooter.feedIn();
  }


  public void rightTriggerCopy() {
    m_shooter.feedIn();
    m_shooter.launch();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Auto1"); // Translation
    // return new PathPlannerAuto("Auto2"); // Turning
  }
}
