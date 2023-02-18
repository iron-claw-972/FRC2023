package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class ManualController {

    GameController manual = new GameController(OIConstants.kDriverJoy);

    public void ElevatorTest()   {
        
    }

    public void FourBarTest(FourBarArm fourbar)  {    
        // elevator controls
        manual.get(Button.Y).onTrue(new InstantCommand(() -> fourbar.test()));
    }
}