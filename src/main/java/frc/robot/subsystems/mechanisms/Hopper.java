// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static frc.robot.utils.Constants.BootyConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  // motor instances
  private final TalonFX m_hopperextend = new TalonFX(k_hopperextendID);

  /** Hopper extension subsystem. */
  public Hopper() {
    m_hopperextend.getConfigurator().apply(k_hopperextendconfig);
  }

  /** Extends the hopper extension. */
  public void extend() {
    m_hopperextend.setVoltage(3);
  }

  /** Retracts the hopper extension. */
  public void retract() {
    m_hopperextend.setVoltage(-3);
  }

  /** Stops the motor. */
  public void stop() {
    m_hopperextend.stopMotor();
  }

  /** Returns the stator current for mechanical stop detection. */
  public double bootyCurrent() {
    return m_hopperextend.getStatorCurrent().getValue().in(Units.Amp);
  }

  @Override
  public void periodic() {
  }
}
