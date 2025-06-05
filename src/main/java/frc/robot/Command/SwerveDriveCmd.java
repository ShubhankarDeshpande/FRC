// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import java.nio.channels.ShutdownChannelGroupException;
import java.util.function.Supplier;

import com.revrobotics.spark.config.SmartMotionConfigAccessor;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCmd extends Command {

  private SwerveSubsystem driveSubsystem;
  private Supplier<Double> xVelFunction;
  private Supplier<Double> yVelFunction; 
  private Supplier<Double> rotVelFunction;

  public ADIS16470_IMU gyro = new ADIS16470_IMU();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // TODO: use individual limiters for each module
  // SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kDirectionSlewRate);
  // SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);

  // base vectors (front left, back right)
  Translation2d baseXVec[] = {new Translation2d(1, 0), new Translation2d(1, 0)};
  Translation2d baseYVec[] = {new Translation2d(0, 1), new Translation2d(0, 1)};
  Translation2d baseRotVec[] = {new Translation2d(-1, 1), new Translation2d(1, -1)};


  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPoseE", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArrayE", Pose2d.struct).publish();
   // Previous angle (so the modules don't rotate in place)
  Rotation2d prevAngle[] = new Rotation2d[] {
    new Rotation2d(0),
    new Rotation2d(0),
    new Rotation2d(0),
    new Rotation2d(0)
  };

  public SwerveDriveCmd(SwerveSubsystem dSub, Supplier<Double> xVel, Supplier<Double> yVel, Supplier<Double> w) {
    driveSubsystem = dSub;
    xVelFunction = xVel;
    yVelFunction = yVel;
    rotVelFunction = w;
    addRequirements(driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d frontLeftVec = baseXVec[0].times(xVelFunction.get()).plus(baseYVec[0].times(yVelFunction.get())).plus(baseRotVec[0].times(rotVelFunction.get()));
    Translation2d frontRightVec = baseXVec[0].times(xVelFunction.get()).plus(baseYVec[0].times(yVelFunction.get())).plus(baseRotVec[0].times(rotVelFunction.get()));
    Translation2d backRightVec = baseXVec[1].times(xVelFunction.get()).plus(baseYVec[1].times(yVelFunction.get())).plus(baseRotVec[1].times(-1 * rotVelFunction.get()));
    Translation2d backLeftVec = baseXVec[1].times(xVelFunction.get()).plus(baseYVec[1].times(yVelFunction.get())).plus(baseRotVec[1].times(-1 * rotVelFunction.get()));

    SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(frontLeftVec, frontRightVec, backLeftVec, backRightVec);
    
    Pose2d posefl = new Pose2d(frontLeftVec.getX(), frontLeftVec.getY(), Rotation2d.fromDegrees(0)); 
    Pose2d posefr = new Pose2d(frontRightVec.getX(), frontRightVec.getY(), Rotation2d.fromDegrees(0)); 
    Pose2d posebr = new Pose2d(backRightVec.getX(), backRightVec.getY(), Rotation2d.fromDegrees(0)); 
    Pose2d posebl = new Pose2d(backLeftVec.getX(), backLeftVec.getY(),Rotation2d.fromDegrees(0)); 
  
    publisher.set(posefl);
    arrayPublisher.set(new Pose2d[] {posefl});
    publisher.set(posefr);
    arrayPublisher.set(new Pose2d[] {posefr});
    publisher.set(posebr);
    arrayPublisher.set(new Pose2d[] {posebr});
    publisher.set(posebl);
    arrayPublisher.set(new Pose2d[] {posebl});

    gyro.setGyroAngleX(frontLeftVec.getAngle().getDegrees());
    SmartDashboard.putNumber("gyro",gyro.getAccelX()); 

    SwerveModuleState desiredStateFrontLeft = new SwerveModuleState(frontLeftVec.getNorm() , frontLeftVec.getAngle().plus(Rotation2d.fromDegrees(135))); 
    SwerveModuleState desiredStateFrontRight = new SwerveModuleState(frontRightVec.getNorm() , frontRightVec.getAngle().plus(Rotation2d.fromDegrees(135))); 
    SwerveModuleState desiredStateBackRight = new SwerveModuleState(backRightVec.getNorm() , backRightVec.getAngle().plus(Rotation2d.fromDegrees(-225))); 
    SwerveModuleState desiredStateBackLeft = new SwerveModuleState(backLeftVec.getNorm() , backLeftVec.getAngle().plus(Rotation2d.fromDegrees(-225)));

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    // Normalize speeds, so the maximum possible speed is 1
    Double maxVel = Math.max(Math.max(desiredStateFrontLeft.speedMetersPerSecond, desiredStateBackRight.speedMetersPerSecond),
                          Math.max(desiredStateFrontRight.speedMetersPerSecond, desiredStateBackLeft.speedMetersPerSecond));
    if (maxVel > 1) {
      desiredStateFrontLeft.speedMetersPerSecond /= maxVel;
      desiredStateFrontRight.speedMetersPerSecond /= maxVel;
      desiredStateBackRight.speedMetersPerSecond /= maxVel;
      desiredStateBackLeft.speedMetersPerSecond /= maxVel;
    }
    desiredStateFrontLeft.speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;
    desiredStateFrontRight.speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;
    desiredStateBackRight.speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;
    desiredStateBackLeft.speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;

  
    if (desiredStateFrontLeft.speedMetersPerSecond <= 0.05) {
      desiredStateFrontLeft.speedMetersPerSecond = 0;
      desiredStateFrontLeft.angle = prevAngle[0];
    }
    if (desiredStateBackRight.speedMetersPerSecond <= 0.05) {
      desiredStateBackRight.speedMetersPerSecond = 0;
      desiredStateBackRight.angle = prevAngle[1];
    }
    if (desiredStateFrontRight.speedMetersPerSecond <= 0.05) {
      desiredStateFrontRight.speedMetersPerSecond = 0;
      desiredStateFrontRight.angle = prevAngle[2];
    }
    if (desiredStateBackLeft.speedMetersPerSecond <= 0.05) {
      desiredStateBackLeft.speedMetersPerSecond = 0;
      desiredStateBackLeft.angle = prevAngle[3];
    }
    prevAngle[0] = desiredStateFrontLeft.angle;
    prevAngle[1] = desiredStateBackRight.angle;
    prevAngle[2] = desiredStateFrontRight.angle;
    prevAngle[3] = desiredStateBackLeft.angle;

    // System.out.println("front left speed " + desiredStateFrontLeft.speedMetersPerSecond);
    // System.out.println("back right speed " + desiredStateBackRight.speedMetersPerSecond);
    // System.out.println("front left rot " + desiredStateFrontLeft.angle.getDegrees());
    // System.out.println("back right rot " + desiredStateBackRight.angle.getDegrees());
 
    SmartDashboard.putNumber("fRONT left vec", frontLeftVec.getAngle().getDegrees());
    SmartDashboard.putNumber("Back Right vec", frontLeftVec.getAngle().getDegrees());
    SmartDashboard.putNumber("Front left x", frontLeftVec.getX());

    SmartDashboard.putNumber("Front Left Speed", desiredStateFrontLeft.speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left Angle", desiredStateFrontLeft.angle.getDegrees());
    SmartDashboard.putNumber("Back Right Speed", desiredStateBackRight.speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Angle", desiredStateBackRight.angle.getDegrees());
    SmartDashboard.putNumber("Front Right Speed", desiredStateFrontRight.speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Angle", desiredStateFrontRight.angle.getDegrees());
    SmartDashboard.putNumber("Back Left Speed", desiredStateBackLeft.speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Angle", desiredStateBackLeft.angle.getDegrees());

    driveSubsystem.setSwerveState(desiredStateFrontLeft, desiredStateBackRight, desiredStateBackLeft, desiredStateFrontRight);
  }


 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}