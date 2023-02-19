package frc.robot.commands.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeAndStop extends SequentialCommandGroup {
    Intake m_intake;

    public IntakeAndStop(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
        addCommands(
                new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed)),
                new WaitUntilCommand(() -> m_intake.isContained()),
                new InstantCommand(() -> m_intake.stop())
        );
    }

}