// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static frc.robot.utils.Constants.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // motor instances
  private final TalonFX m_intakerollers = new TalonFX(k_intakerollerID);

  /** Subsystem that controls the intake/agitator assembly. */
  public Intake() {
    m_intakerollers.getConfigurator().apply(k_intakerollerconfig);
  }

  /** Runs the intake and agitator forward. */
  public void intake() {
    m_intakerollers.setVoltage(9);
  }

  /** Runs the intake in revers to clear jams or dump game pieces. Maybe.*/
  public void outtake() {
    m_intakerollers.setVoltage(-9);
  }

  /** Stops the motor. */
  public void stop() {
    m_intakerollers.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
