// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class LLVision extends SubsystemBase {

  public LLVision() {}

  /* Gets the x, y ,and angle of tracked apriltag */
  public double[] getApriltagCoordinates(){
    return new double[]{
      LimelightHelpers.getTX("limelight"), 
      LimelightHelpers.getTY("limelight"), 
      LimelightHelpers.getTA("limelight")
      };
  }

  /* Gets the X and Y position of the apriltag being tracked, applies kP and Offsets, and returns a double array for use with swerve drive */
  public double[] aimAndRange(){
    double kP = 0.035;

    double angularVel = LimelightHelpers.getTX("limelight") * kP;
    double rawForwardSpeeds = LimelightHelpers.getTY("limelight");

    double error = 3.0 - rawForwardSpeeds;

    if (Math.abs(error) < 0.05){
      error = 0.0;
    }

    double forwardSpeeds = error * kP;
    return new double[] {
      forwardSpeeds,
      angularVel
    };


  }

  //@Override
  public void periodic() {
    aimAndRange();
  }
}
