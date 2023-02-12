// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.LinkedList;
import java.util.List;

/** Add your docs here. */
public class FeedForwardCharacterizationData {
  PolynomialRegression m_regression;
  private final List<Double> velocityData = new LinkedList<>();
  private final List<Double> voltageData = new LinkedList<>();

  

  public void add(double velocity, double voltage) {
    if (Math.abs(velocity) > 1E-4) {
      velocityData.add(Math.abs(velocity));
      voltageData.add(Math.abs(voltage));
    }
  }

  public void print() {
    m_regression = new PolynomialRegression(
        velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
        voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
        1);

    
  }
  public double getStatic(){
      return m_regression.beta(0);
  }
  public double getVelocity(){
      return m_regression.beta(1);
  }
  public double getVariance(){
    return m_regression.R2();
  }
}

