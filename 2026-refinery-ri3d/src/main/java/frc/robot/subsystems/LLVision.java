// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class LLVision extends SubsystemBase {
  private final PIDController rangeController = new PIDController(0.1, 0, 0);
  private final PIDController aimController = new PIDController(0.1, 0, 0);

  private static final double finalDistance = 3.0; //vertical offset
  
  public LLVision() {
    //tolerances
    rangeController.setTolerance(0.05); //max error in centimeters
    aimController.setTolerance(1.5); //max error in degrees
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

  public double[] aimAndRange2() {
    //get apriltag values from limelight
    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");

    //check if values exist
    if (!LimelightHelpers.getTV("limelight")) {
      return new double[] {
        0.0,
        0.0
      };
    }

    //pid output
    double angularVel = aimController.calculate(tx, 0.0);
    double forwardSpeeds = rangeController.calculate(ty, finalDistance);

    //clamps the final output to a range between -1.0 and 1.0 for less lurching when auto aiming
    angularVel = MathUtil.clamp(angularVel, -1.0, 1.0);
    forwardSpeeds = MathUtil.clamp(forwardSpeeds, -1.0, 1.0);

    return new double[] {
      forwardSpeeds,
      angularVel
    };
  }

  @Override
  public void periodic() {
    //aimAndRange();
    aimAndRange2();
  }
}
