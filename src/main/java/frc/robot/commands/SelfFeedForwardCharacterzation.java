// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.ShuffleboardManager;
import frc.robot.subsystems.Module;

/** Add your docs here. */
public class SelfFeedForwardCharacterzation extends CommandBase {
  double value = 0;
  FeedForwardCharacterizationData m_feedForwardCharacterizationData0 = new FeedForwardCharacterizationData(
      Robot.drive.m_modules[0]);
  FeedForwardCharacterizationData m_feedForwardCharacterizationData1 = new FeedForwardCharacterizationData(
      Robot.drive.m_modules[1]);
  FeedForwardCharacterizationData m_feedForwardCharacterizationData2 = new FeedForwardCharacterizationData(
      Robot.drive.m_modules[2]);
  FeedForwardCharacterizationData m_feedForwardCharacterizationData3 = new FeedForwardCharacterizationData(
      Robot.drive.m_modules[3]);

  Timer m_timer = new Timer();
  Drivetrain m_drive;

  public SelfFeedForwardCharacterzation(Drivetrain drive) {
    this.m_drive = drive;

    addRequirements(drive);
  }

  public void initialize() {
    m_timer.start();
    m_drive.setAllOptimize(false);
  }

  public void execute() {
    runcharacterazationVolts();
    if (m_timer.get() > 2.0) {
      m_feedForwardCharacterizationData0.add(Robot.drive.m_modules[0].getDriveVelocity(), value);
      m_feedForwardCharacterizationData1.add(Robot.drive.m_modules[1].getDriveVelocity(), value);
      m_feedForwardCharacterizationData2.add(Robot.drive.m_modules[2].getDriveVelocity(), value);
      m_feedForwardCharacterizationData3.add(Robot.drive.m_modules[3].getDriveVelocity(), value);
    }
    if (m_timer.get() >= 4.0) {
      value += 0.2;
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

  private double getValue() {
    return value;
  }

  private void updatelogs() {
    LogManager.addDouble("Volt", () -> getValue());
    LogManager.addDouble("Vel Front Left Raw", () -> Robot.drive.m_modules[0].getDriveVelocity());
    LogManager.addDouble("Vel Front Right Raw", () -> Robot.drive.m_modules[1].getDriveVelocity());
    LogManager.addDouble("Vel Back Left Raw", () -> Robot.drive.m_modules[2].getDriveVelocity());
    LogManager.addDouble("Vel Back Right Raw", () -> Robot.drive.m_modules[3].getDriveVelocity());
  }

  public void end(Boolean interrupted) {
    m_feedForwardCharacterizationData0.print();
    m_feedForwardCharacterizationData1.print();
    m_feedForwardCharacterizationData2.print();
    m_feedForwardCharacterizationData3.print();

  }

  public Boolean isFinsihed() {
    return value > 11;
  }

  public static class FeedForwardCharacterizationData {
    private final Module name;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public FeedForwardCharacterizationData(Module m_modules) {
      this.name = m_modules;
    }

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      PolynomialRegression regression = new PolynomialRegression(
          velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
          voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
          1);

      // System.out.println("FF Characterization Results (" + name + "):");
      // System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      // System.out.println(String.format("\tR2=%.5f", regression.R2()));
      // System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      // System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
      Robot.shuffleboard.m_staticModulesSaver.replace(name, regression.beta(0));
      Robot.shuffleboard.m_staticModulesSaver.replace(name, regression.beta(1));
    }
  }
}
