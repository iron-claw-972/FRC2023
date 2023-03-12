package frc.robot.commands.scoring.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class RotateWrist extends CommandBase {
    private final Wrist m_wrist;
    private double m_wristSetpoint;
  
    /**
     * rotates wrist to an angle using PID
     */
    public RotateWrist(Wrist wrist, double setpoint) {
        addRequirements(wrist);
        m_wrist = wrist;
        m_wristSetpoint = setpoint;
    }

    @Override
    public void initialize() {
        m_wrist.setEnabled(true);
        m_wrist.setArmSetpoint(m_wristSetpoint);
    }

    @Override
    public boolean isFinished() {
        return m_wrist.reachedSetpoint();
    }
}
