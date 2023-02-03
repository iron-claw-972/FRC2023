package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.controls.Operator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveElevatorWithJoystick extends CommandBase {
  AngledElevator m_elevator; 
  DigitalInput m_topSwitch; //TODO: Determine if this is the thing we are using to determine top and bottom elevator height
  DigitalInput m_bottomSwitch;
  WPI_TalonFX m_elevatorMotor; 


  public MoveElevatorWithJoystick(DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch, AngledElevator elevator, WPI_TalonFX elevatorMotor) {
    m_topSwitch = topLimitSwitch; 
    m_bottomSwitch = bottomLimitSwitch;
    m_elevator = elevator; 
    m_elevatorMotor = elevatorMotor; 
    addRequirements(m_elevator);
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevatorMotor.set(Operator.getRawThrottleValue());
  
    if (m_topSwitch.get() || m_bottomSwitch.get()){ //Is the right way of writing or? 
      m_elevatorMotor.set(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
