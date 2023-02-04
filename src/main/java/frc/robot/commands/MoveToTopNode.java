package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AngledElevator;

public class MoveToTopNode extends CommandBase {
  AngledElevator m_elevator; 
  WPI_TalonFX m_elevatorMotor; 
  PIDController m_moveToTopNodePid; 


  public MoveToTopNode(AngledElevator elevator, WPI_TalonFX elevatorMotor, PIDController moveToTopNodePid) {
    
    m_elevator = elevator; 
    m_elevatorMotor = elevatorMotor; 
    m_moveToTopNodePid = moveToTopNodePid; 
    addRequirements(m_elevator);
  
  }

  @Override
  public void initialize() {
      
  }

  @Override
  public void execute() {
    double pid = MathUtil.clamp(m_moveToTopNodePid.calculate(m_elevator.getElevatorHeightInches(), Constants.elevator.kTopNodeSetpoint),-0.25,0.25);
    double error = Constants.elevator.kTopNodeSetpoint-m_elevator.getElevatorHeightInches(); 
    //TODO: Increase clamping range to make PID go faster

    while(error <=-5 || error >=5){ //TODO: Fix error range if needed
     
      m_elevatorMotor.set(pid); 

    }

    if(error >=-5 || error <=5){
      
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
