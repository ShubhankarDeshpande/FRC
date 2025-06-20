// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Command.ElevatorRunnerCommands;
import frc.robot.Command.SwerveDriveCmd;
import frc.robot.Command.SwerveDriveCmd;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.time.Instant;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ElevatorSubsystem m_intakeMotor = new ElevatorSubsystem(Constants.IntakeConstants.kIntakeMotorCanId);
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  // The driver's controller

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveDriveCmd(
      swerveSubsystem,
       () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
       () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
       () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
       () -> m_driverController.getRawAxis(OIConstants.kDriverJoystickRotAxis)
       ));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    /* m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              true),
          m_robotDrive)); */
    configureButtonBindings();
            

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

    new JoystickButton(m_driverController, XboxController.Button.kA.value) // when this is true, say its pressed and make a new intake runner command
   .onTrue(new InstantCommand(() -> { 
        System.out.println("its pressed" );
        new ElevatorRunnerCommands(m_intakeMotor, 0.5).schedule();//you have to schedule the command cuz the framework requires it to be scheduled to use execute()
   }));

   new JoystickButton(m_driverController, XboxController.Button.kA.value) // when this is true, say its pressed and make a new intake runner command
   .onFalse(new InstantCommand(() -> { 
      if(m_intakeMotor.getPower() > 0){
        System.out.println("its released" );
        new ElevatorRunnerCommands(m_intakeMotor, -0.5).schedule();//you have to schedule the command cuz the framework requires it to be scheduled to use execute()
      }
   }));
    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
