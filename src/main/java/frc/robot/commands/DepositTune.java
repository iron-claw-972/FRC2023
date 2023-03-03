package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;

public class DepositTune extends CommandBase {
  Elevator m_elevator;
  FourBarArm m_arm;

  double m_height = 0.0;
  double m_position = 0.0;

  boolean m_elevatorDone = false;

  /**
   * Moves the elevator and arm to the positions indicated by Smartdashboard
   * @param elevator the elevator subsystem
   * @param arm the arm subsystem
   */
  public DepositTune(Elevator elevator, FourBarArm arm) {
    m_elevator = elevator; 
    m_arm = arm;

    addRequirements(elevator, arm);
  }

  @Override
  public void initialize() {
    // get the parameters from the SmartDashboard
    m_height = SmartDashboard.getNumber("Deposit Elevator Extension", 0);
    m_position = SmartDashboard.getNumber("Deposit Arm Extension", 0);

    // make sure the parameters are sane
    m_height = MathUtil.clamp(m_height, 0, ElevatorConstants.kMaxPosition);
    m_position = MathUtil.clamp(m_position, ArmConstants.kStowPos, ArmConstants.kMaxArmExtensionPos); 

    // report the trimmed values on the dashboard
    SmartDashboard.putNumber("Deposit Elevator Extension", m_height);
    SmartDashboard.putNumber("Deposit Arm Extension", m_position);

    // move the elevator to the desired position
    m_elevator.setDesiredPosition(m_height);

    m_elevatorDone = false;
  }

  @Override
  public void execute(){
    // implement a trivial sequential command
    if(!m_elevatorDone && m_elevator.reachedDesiredPosition()){
      m_elevatorDone = true; 
      m_arm.setArmSetpoint(m_position);
    }
  }

  @Override
  public boolean isFinished(){
    return (m_elevatorDone && m_arm.reachedSetpoint()); 
  }
}