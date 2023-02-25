package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class TestController {
  
  private GameController test = new GameController(OIConstants.kTestJoy);
  private FourBarArm m_arm;
  private Intake m_intake;
  private Drivetrain m_drive;
  
  public TestController(FourBarArm arm, Intake intake, Drivetrain drive){
    m_arm = arm;
    m_intake = intake;
    m_drive = drive;
  }
  
  public void configureControls() {
    
    // elevator controls
    test.get(Button.Y).onTrue(new ExtendToPosition(m_arm, ArmConstants.kTopPosition));
    test.get(Button.X).onTrue(new ExtendToPosition(m_arm, ArmConstants.kMiddlePosition));
    test.get(Button.A).onTrue(new ExtendToPosition(m_arm, ArmConstants.kIntakePosition));
    test.get(Button.B).onTrue(new ExtendToPosition(m_arm, ArmConstants.kShelfPosition));
    
    // intake controls
    test.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
    test.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed),m_intake));
    test.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stop(), m_intake));

    //balance command
    test.get(GameController.Button.RB).onTrue(new BalanceCommand(m_drive));
  }
}

