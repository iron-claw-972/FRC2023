package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

public class StowGamePiece extends SequentialCommandGroup {
    private final Intake intake;
    private final double intakeSpeed;
    private final double armSpeed;

    public StowGamePiece(Intake intake, double intakeSpeed, double armSpeed) {
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;
        this.armSpeed = armSpeed;

        addCommands(
            new MoveIntakeCommand(intake, intakeSpeed),
            new MoveArmCommand(intake, armSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        intake.stopArm();
    }
}
