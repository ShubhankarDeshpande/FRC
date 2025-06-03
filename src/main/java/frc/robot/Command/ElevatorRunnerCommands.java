package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRunnerCommands extends Command{
    private double power;
    private final ElevatorSubsystem m_intake;
  

    public ElevatorRunnerCommands(ElevatorSubsystem m_intake){
        addRequirements(m_intake);
        this.m_intake = m_intake;
  
    }
    
    public ElevatorRunnerCommands(ElevatorSubsystem m_intake, double power){
        addRequirements(m_intake);
        this.m_intake = m_intake;
        this.power = power;
      
    }

    public void initialize() {
        m_intake.runIntake(power);
    }
   
    // runs every 3 milliseconds
    
    public void execute(){
      
        if(this.power >= 0.5){
            m_intake.runIntake(power+1);
            power += 1;
        }
       
        if(this.power <= -0.5){
            m_intake.runIntake(power-1);
            power -= 1;
        }
        if(m_intake.getPosition() == 0){
            power = 0;
        }
        
    }

    //calls end with true if program was interrupted
    public void end(boolean interrupted){
        m_intake.runIntake(0);
    }

    //gives a condition if the command was run
    public boolean isFinished() {
        return false;
    }
}
