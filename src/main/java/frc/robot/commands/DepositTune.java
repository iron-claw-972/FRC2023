package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.Stow;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.elevator.MoveToExtension;
import frc.robot.commands.intake.OuttakeNormal;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class DepositTune extends CommandBase {
  Elevator m_elevator; 
  FourBarArm m_arm;
  Intake m_intake;

  double m_height = 0.0;
  double m_position = 0.0;

  boolean m_elevatorDone = false; 

  public DepositTune(Elevator elevator, FourBarArm arm, Intake intake) {
    m_elevator = elevator; 
    m_arm = arm; 
    m_intake = intake; 

    addRequirements(elevator, arm, intake);
  }

  @Override
  public void initialize() {
    // get the parameters from the SmartDashboard
    m_height = SmartDashboard.getNumber("DepositHeight", 0);
    m_position = SmartDashboard.getNumber("DepositPosition", 0);

    // make sure the parameters are sane
    m_height = MathUtil.clamp(m_height, ElevatorConstants.kMinExtension, ElevatorConstants.kMaxExtension);
    m_position = MathUtil.clamp(m_position, ArmConstants.kInitialPosition, ArmConstants.kTopPosition); 

    // repot the trimmed values on the dashboard
    SmartDashboard.putNumber("DepositHeight", m_height);
    SmartDashboard.putNumber("DepositPosition", m_position);

    // move the elevator to the desired position
    m_elevator.setTargetExtension(m_height);

    m_elevatorDone = false;
  }

  @Override
  public void execute(){
    // implement a trivial sequential command
    if(!m_elevatorDone && m_elevator.atSetpoint()){
      m_elevatorDone = true; 
      m_arm.setArmSetpoint(m_position);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
  }
  
  @Override
  public boolean isFinished(){
    return (m_elevatorDone && m_arm.reachedSetpoint()); 
  }


}



