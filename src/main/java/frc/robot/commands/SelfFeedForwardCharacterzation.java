// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.LogManager;

/** Add your docs here. */
public class SelfFeedForwardCharacterzation extends CommandBase{
    double value =0;
    Timer m_timer=new Timer();
    public void initialize(){
      m_timer.start();
      updatelogs();

    }
    public void execute(){
      
        
            
          runcharacterazationVolts();
            if (m_timer.get()>=2.0){
            }

            if (m_timer.get()>=4.0){
                value+=0.2;
                m_timer.reset();
                m_timer.start();
            }
                

        
        

    }
    private void runcharacterazationVolts() {
        for (int i = 0; i < 4; i++) {
          Robot.drive.m_modules[i].setDriveVoltage(0);
          Robot.drive.m_modules[i].setSteerVoltage(value);
        }
      }
      private double getValue(){
        return value;
      }
      private void updatelogs(){
        LogManager.addDouble("Volt", ()->getValue());
        LogManager.addDouble("Vel Front Left Raw", () -> Robot.drive.m_modules[0].getDriveVelocity());
        LogManager.addDouble("Vel Front Right Raw", () -> Robot.drive.m_modules[1].getDriveVelocity());
        LogManager.addDouble("Vel Back Left Raw", () -> Robot.drive.m_modules[2].getDriveVelocity());
        LogManager.addDouble("Vel Back Right Raw", () -> Robot.drive.m_modules[3].getDriveVelocity());
      }
      public Boolean isFinsihed(){
        return value>11;
      }
}
