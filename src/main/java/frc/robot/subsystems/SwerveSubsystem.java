package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
    private MAXSwerveModule m_SwerveModuleFrontLeft;
    //private MAXSwerveModule m_SwerveModuleBackLeft;
    //private MAXSwerveModule m_SwerveModuleFrontRight;
    private MAXSwerveModule m_SwerveModuleBackRight;

    private SwerveModuleState[] allState = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    //for visualization
    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();

    public SwerveSubsystem() {   
        // Initialize the swerve module with the CAN IDs and angular offset
        m_SwerveModuleFrontLeft = new MAXSwerveModule(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
        //m_SwerveModuleBackLeft = new MAXSwerveModule(DriveConstants.kBackLeftDriveCanId, DriveConstants.kBackLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
        //m_SwerveModuleFrontRight = new MAXSwerveModule(DriveConstants.kFrontRightDriveCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
        m_SwerveModuleBackRight = new MAXSwerveModule(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);

    }
    
    public void setSwerveState(SwerveModuleState stateFrontLeft, SwerveModuleState stateBackRight /*, SwerveModuleState stateBackLeft, SwerveModuleState stateFrontRight*/) {
        //SwerveModuleState desiredState = new SwerveModuleState();
        //sets the speed and angle of the desired state to the current state
        //desiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        //desiredState.angle = state.angle;

        allState = new SwerveModuleState[] {
            stateFrontLeft,
            //SwerveModuleState stateBackLeft
            new SwerveModuleState(),
            //SwerveModuleState stateBackLeft
            new SwerveModuleState(),
            stateBackRight
        };
        m_SwerveModuleFrontLeft.setDesiredState(stateFrontLeft);
        m_SwerveModuleFrontLeft.setDesiredState(stateBackRight);
        //m_SwerveModuleBackLeft.setDesiredState(stateBackLeft);
        //m_SwerveModuleFrontRight.setDesiredState(stateFrontRight);
    
    }

    public void resetEncoders(){
        m_SwerveModuleFrontLeft.resetEncoders();
        m_SwerveModuleBackRight.resetEncoders();
        //m_SwerveModuleBackLeft.resetEncoders();
        //m_SwerveModuleFrontRight.resetEncoders();

    }

    public SwerveModuleState getFrontLeftState(){
        return m_SwerveModuleFrontLeft.getState();
    }

    public SwerveModuleState getBackRightState(){
        return m_SwerveModuleBackRight.getState();
    }

    @Override
    public void periodic() {
        publisher.set(allState);
    }

    @Override
    public void simulationPeriodic() {

    }



}