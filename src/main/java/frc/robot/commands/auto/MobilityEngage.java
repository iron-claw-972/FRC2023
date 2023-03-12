package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class MobilityEngage extends SequentialCommandGroup {

  /**
   * Drives the robot over the charge station. Automatically detects when done using pitch.
   * @param drive
   */
  public MobilityEngage(Drivetrain drive) {
    addRequirements(drive);
    addCommands(
      // run the drivetrain
      new InstantCommand(() -> drive.drive(DriveConstants.kDriveChargeStationSpeed, 0, 0, true, false), drive),
      // will first wait until pitch is negative, then until pitch is zero. 
      new WaitUntilCommand(() -> drive.getPitch().getDegrees() < -8),
      new WaitUntilCommand(() -> Math.abs(drive.getPitch().getDegrees()) < 0.5),
      new WaitCommand(1.5),
      new InstantCommand(() -> drive.drive(-2, 0, 0, true, false), drive),
      new WaitUntilCommand(() -> drive.getPitch().getDegrees() > 8),
      new BalanceCommand(drive)
    );
  }
}
