package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AngledElevator;

public class ElevatorCalibration extends CommandBase {
  AngledElevator m_elevator; 
  DigitalInput m_topLimitSwitch; 
  DigitalInput m_bottomLimitSwitch; 
  WPI_TalonFX m_elevatorMotor; 

  public ElevatorCalibration(AngledElevator elevator, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch, WPI_TalonFX elevatorMotor) {
    
    m_elevator = elevator; 
    m_topLimitSwitch = topLimitSwitch; 
    m_bottomLimitSwitch = bottomLimitSwitch; 
    m_elevatorMotor = elevatorMotor; 

    addRequirements(m_elevator);
  
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {

    while(m_bottomLimitSwitch.get() == false){
       
      m_elevatorMotor.set(-0.25); 
  
    }

    if(m_bottomLimitSwitch.get()){

      m_elevatorMotor.set(0); 
      m_elevatorMotor.setNeutralMode(NeutralMode.Brake);//unsure if this is neccessary or not
      m_elevator.resetEncoders(); 
      m_elevatorMotor.set(0.25); 

    }

    if(m_topLimitSwitch.get()){

      Constants.elevator.kElevatorTopHeightInches = m_elevator.getElevatorHeightInches(); //TODO: Put this value on shuffleboard 
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
