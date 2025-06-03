package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Micro;

import java.security.PrivateKey;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax m_intakeMotor;
    private final RelativeEncoder m_encoder;
    private double m_power;
    private double maxPosition = 10.1;
    private double minPosition = -0.01;
    private double position = 0;

    public ElevatorSubsystem(int motorid){
        m_intakeMotor = new SparkMax(motorid, SparkMax.MotorType.kBrushless);
        m_encoder = m_intakeMotor.getEncoder(); //make a new encoder object of the sparkmax motor to track rpm
        m_encoder.setPosition(0);

    }

    public double getPosition() {
        return m_encoder.getPosition(); //gets the position of the encoder
    }

    public void runIntake(double power){
        this.m_power = power;
        m_intakeMotor.set(power);
        if(m_encoder.getPosition() >= maxPosition){ //basically if the encoder detects that its past 10 rotations, it sets the speed to zero, stopping the motor
            m_intakeMotor.set(0);
            m_encoder.setPosition(maxPosition);

        }
        if(m_encoder.getPosition() <= minPosition + 0.01){ //basically if the encoder detects that its past 10 rotations, it sets the speed to zero, stopping the motor
            m_intakeMotor.set(0);
            m_encoder.setPosition(minPosition);

        }
      
    }

    public double getPower(){
        return m_power;
    }
    @Override
    public void simulationPeriodic(){
        position += m_power * 0.02 * 10;
        m_encoder.setPosition(position);
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        if (m_encoder.getPosition() > maxPosition) {
            position = maxPosition;
            m_encoder.setPosition(maxPosition);
       //     m_power = 0;
         //   m_intakeMotor.set(m_power);
            SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());


        } 
        if (m_encoder.getPosition() < minPosition) {
            position = minPosition;
            m_encoder.setPosition(minPosition);
          //  m_power = 0;
            //m_intakeMotor.set(m_power);
            SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());



        }
       
    }
}
