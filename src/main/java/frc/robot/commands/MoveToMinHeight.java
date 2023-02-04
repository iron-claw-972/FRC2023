package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToMinHeight extends CommandBase {
  AngledElevator m_elevator; 
  DigitalInput m_bottomLimitSwitch; 
  WPI_TalonFX m_elevatorMotor; 

  public MoveToMinHeight(AngledElevator elevator, DigitalInput bottomLimitSwitch, WPI_TalonFX elevatorMotor) {
    
    m_elevator = elevator; 
    m_bottomLimitSwitch = bottomLimitSwitch; 
    m_elevatorMotor = elevatorMotor; 

    addRequirements(m_elevator);
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevatorMotor.set(0.25); 

    if(m_bottomLimitSwitch.get()){
      m_elevatorMotor.set(0); 
      m_elevatorMotor.setNeutralMode(NeutralMode.Brake); //unsure if this is neccessary or not
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
