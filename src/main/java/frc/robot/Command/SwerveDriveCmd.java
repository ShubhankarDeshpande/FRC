// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCmd extends Command {

  private SwerveSubsystem driveSubsystem;
  private Supplier<Double> xVelFunction;
  private Supplier<Double> yVelFunction;
  private Supplier<Double> rotVelFunction;

  // TODO: use individual limiters for each module
  // SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kDirectionSlewRate);
  // SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);

  // base vectors (front left, back right)
  Translation2d baseXVec[] = {new Translation2d(1, 0), new Translation2d(1, 0)};
  Translation2d baseYVec[] = {new Translation2d(0, 1), new Translation2d(0, 1)};
  Translation2d baseRotVec[] = {new Translation2d(-1, 1), new Translation2d(1, -1)};

   // Previous angle (so the modules don't rotate in place)
  Rotation2d prevAngle[] = new Rotation2d[] {
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
    Translation2d backRightVec = baseXVec[1].times(xVelFunction.get()).plus(baseYVec[1].times(yVelFunction.get())).plus(baseRotVec[1].times(rotVelFunction.get()));

    SwerveModuleState desiredStateFrontLeft = new SwerveModuleState(frontLeftVec.getNorm(), frontLeftVec.getAngle());
    SwerveModuleState desiredStateBackRight = new SwerveModuleState(backRightVec.getNorm(), backRightVec.getAngle());

    // Normalize speeds, so the maximum possible speed is 1
    Double maxVel = Math.max(desiredStateFrontLeft.speedMetersPerSecond, desiredStateBackRight.speedMetersPerSecond);
    if (maxVel > 1) {
      desiredStateFrontLeft.speedMetersPerSecond /= maxVel;
      desiredStateBackRight.speedMetersPerSecond /= maxVel;
    }
    desiredStateFrontLeft.speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;
    desiredStateBackRight.speedMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond;

    // Slew rate limiters (THIS IS BROKEN)
    // TODO: Use individual limiters for each module
    // desiredStateFrontLeft.speedMetersPerSecond = magLimiter.calculate(desiredStateFrontLeft.speedMetersPerSecond);
    // desiredStateBackRight.speedMetersPerSecond = magLimiter.calculate(desiredStateBackRight.speedMetersPerSecond);
    // desiredStateFrontLeft.angle = new Rotation2d(rotLimiter.calculate(desiredStateFrontLeft.angle.getRadians()));
    // desiredStateBackRight.angle = new Rotation2d(rotLimiter.calculate(desiredStateBackRight.angle.getRadians()));

    // Angle offsers (NOT NEEDED)
    // desiredStateFrontLeft.angle = desiredStateFrontLeft.angle.plus(new Rotation2d(DriveConstants.kFrontLeftChassisAngularOffset));
    // desiredStateBackRight.angle = desiredStateBackRight.angle.plus(new Rotation2d(DriveConstants.kBackRightChassisAngularOffset));
    
    // Prevent rotation in place
    if (desiredStateFrontLeft.speedMetersPerSecond <= 0.05) {
      desiredStateFrontLeft.speedMetersPerSecond = 0;
      desiredStateFrontLeft.angle = prevAngle[0];
    }
    if (desiredStateBackRight.speedMetersPerSecond <= 0.05) {
      desiredStateBackRight.speedMetersPerSecond = 0;
      desiredStateBackRight.angle = prevAngle[1];
    }
    prevAngle[0] = desiredStateFrontLeft.angle;
    prevAngle[1] = desiredStateBackRight.angle;

    // System.out.println("front left speed " + desiredStateFrontLeft.speedMetersPerSecond);
    // System.out.println("back right speed " + desiredStateBackRight.speedMetersPerSecond);
    // System.out.println("front left rot " + desiredStateFrontLeft.angle.getDegrees());
    // System.out.println("back right rot " + desiredStateBackRight.angle.getDegrees());

    driveSubsystem.setSwerveState(desiredStateFrontLeft, desiredStateBackRight);
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