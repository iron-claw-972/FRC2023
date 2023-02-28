/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX m_motor;
  private final PIDController m_elevatorPID;
  private final TalonEncoder m_talonEncoder; 
  private final DigitalInput m_topLimitSwitch; 
  private final DigitalInput m_bottomLimitSwitch; 
  private final ShuffleboardTab m_elevatorTab; 
  private boolean m_enabled; 
  private boolean m_isCalibrated = false; 

  public Elevator(ShuffleboardTab elevatorTab) {
    m_elevatorTab = elevatorTab; 
    m_motor = MotorFactory.createTalonFX(ElevatorConstants.kMotorPort, Constants.kCanivoreCAN);
    m_motor.setInverted(true);
    m_talonEncoder = new TalonEncoder(m_motor); 
    m_talonEncoder.setDistancePerPulse(ElevatorConstants.kDistPerPulse);
    addChild("motor", m_motor);
   
    m_elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    //m_elevatorTab.add("pid",m_elevatorPID);
    //m_elevatorTab.addNumber("",()-> getHeight());
    m_elevatorPID.setTolerance(0.03);

    m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort); 
    m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort); 
    addChild("Upper Limit",m_topLimitSwitch);
    addChild("Bottom Limit",m_bottomLimitSwitch);


    //TODO: log, addDouble doesn't work. 
    //LogManager.addDouble("Elevator/error", () -> {return m_pid.getSetpoint() - getElevatorHeight();});
    
    //m_motor.setSafetyEnabled(true);
    setUpElevatorTab();
    setTargetExtension(0.1); 
  }

  @Override
  public void periodic() {
    if(m_enabled && m_isCalibrated) {
      double pid = m_elevatorPID.calculate(getElevatorExtension());
      double pidClamped = MathUtil.clamp(pid, -ElevatorConstants.kPowerLimit, ElevatorConstants.kPowerLimit);
      double finalMotorPower = pidClamped; 
      
      setMotorPower(finalMotorPower);
    } else {
      //m_motor.feed();
    }
  }

  public void close() {
    // close the ports
    //TODO: Is there any way to close the ports of the limit switches attached to the motor? 
    m_topLimitSwitch.close(); 
    m_bottomLimitSwitch.close(); 
  }

  /**
   * Set the motor power while taking the limit switches into account. 
   * If the bottom limit switch is tripped, then the elevator is only allowed to move up. 
   * If the top limit switch is tripped, then the elevator is inly allowed to move down. 
   * @param power the power we want to give to the motor
   */
  public void setMotorPower(double power) {
    if ( (isBottomSwitchTripped() && power < 0) || (isTopSwitchTripped() && power > 0) ){
      m_motor.set(0); 
    } else {
      m_motor.set(power);
    }
  }
  
  public void setPIDEnabled(boolean isEnabled){
    m_enabled = isEnabled; 
  }

  public void setIsCalibrated(){
    m_isCalibrated = true; 
  }

  public void setTargetExtension(double setpoint){
    m_elevatorPID.setSetpoint(setpoint);
    m_elevatorPID.reset(); 
  }

  public boolean isBottomSwitchTripped(){
    return !m_bottomLimitSwitch.get();
  }

  public boolean isTopSwitchTripped(){
    return !m_topLimitSwitch.get();
  }

  public void resetTalonEncoder(){
    m_talonEncoder.reset(); 
  }
 
  /**
   * Determine the elevator extension from the motor encoder.
   * @return return extension in meters
   */
  public double getElevatorExtension() {
    return m_talonEncoder.getDistance(); 
  }

  public boolean atSetpoint() {
    return m_elevatorPID.atSetpoint();
  }

  public void setUpElevatorTab(){
    m_elevatorTab.addNumber("Elevator Height", () -> getElevatorExtension());
    m_elevatorTab.add(m_elevatorPID); 
    m_elevatorTab.addBoolean("enabled", () -> m_enabled);
    m_elevatorTab.addBoolean("topLimitSwitch", () -> isTopSwitchTripped() );
    m_elevatorTab.addBoolean("bottomLimitSwitch", () -> isBottomSwitchTripped());

  }
}

