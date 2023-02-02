package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveToMaxHeight;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;


public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {

     //bind controller actions such as as pushing buttons/pushing joysticks to actions/commands
     
     //ex:  driver.get(Button.A).whenPressed(new BangBang(drive,20000));

  }

 
}
