// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;

/** Add your docs here. */
public class SlewRateLimiter2D {
    Translation2d m_prevTranslation;
    Translation2d m_AccelTranslation;
    Translation2d m_prevTranslation2d = new Translation2d(0, 0);
    private double m_prevRot = 0;
    private double xVel = 0;
    private double yVel = 0;
    private double rotVel = 0;
    public void calculate(double xSpeed, double ySpeed, double rot){
        double factor = 1;
        Translation2d m_Translation2d = new Translation2d(xSpeed, ySpeed);
        Translation2d m_AccelTranslation = m_Translation2d.minus(m_prevTranslation2d).div(Constants.kLoopTime);
        double rotAccel = (rot-m_prevRot)/Constants.kLoopTime * ((DriveConstants.kTrackWidth/2) * Math.sqrt(2));
        if (m_AccelTranslation.getNorm() + Math.abs(rotAccel) > DriveConstants.kTranLim){
          factor = DriveConstants.kTranLim/ m_AccelTranslation.getNorm() + Math.abs(rotAccel);
        }
        xVel = m_AccelTranslation.getX() * factor * Constants.kLoopTime + m_prevTranslation2d.getX();
        yVel = m_AccelTranslation.getY() * Constants.kLoopTime * factor + m_prevTranslation2d.getY();
        rotVel = rotAccel/((DriveConstants.kTrackWidth/2) * Math.sqrt(2))*Constants.kLoopTime * factor + m_prevRot;
    }
    public double getXvel(){
        return xVel;
    }
    public double getYvel(){
        return yVel;
    }
    public double getRotvel(){
        return rotVel;
    }

}
