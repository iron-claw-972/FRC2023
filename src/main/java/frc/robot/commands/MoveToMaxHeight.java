package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToMaxHeight extends CommandBase {
  AngledElevator m_elevator; 
  DigitalInput m_topLimitSwitch; 
  WPI_TalonFX m_elevatorMotor; 

  public MoveToMaxHeight(AngledElevator elevator, DigitalInput topLimitSwitch, WPI_TalonFX elevatorMotor) {
    
    m_elevator = elevator; 
    m_topLimitSwitch = topLimitSwitch; 
    m_elevatorMotor = elevatorMotor; 

    addRequirements(m_elevator);
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevatorMotor.set(0.25); 

    if(m_topLimitSwitch.get()){
      m_elevatorMotor.set(0); 
      m_elevatorMotor.setNeutralMode(NeutralMode.Brake); 
    }
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

        //TODO: configure isFinished to get command to end

        
    return false;
  }

}
