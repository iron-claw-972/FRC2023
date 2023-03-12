package frc.robot.commands.scoring;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class WristDunk extends SequentialCommandGroup {
    /**
     * Moves the wrist down and releases the intake, to allow dunking the cone on the node
     */
    public WristDunk(Wrist wrist, Intake intake) {
        addRequirements(wrist, intake);
        addCommands(
            new RotateWrist(wrist, WristConstants.kDunkPos),
            new InstantCommand(() -> intake.setIdleMode(IdleMode.kCoast))
        );
    }
}
