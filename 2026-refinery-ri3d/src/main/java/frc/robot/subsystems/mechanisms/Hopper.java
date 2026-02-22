// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.BootyConstants.k_hopperextendID;
import static frc.robot.utils.Constants.BootyConstants.k_hopperextendconfig;
import static frc.robot.utils.Constants.BootyConstants.k_rumblePakID;
import static frc.robot.utils.Constants.BootyConstants.k_rumblepakconfig;

public class Hopper extends SubsystemBase {

  // motor instances
  private final TalonFX m_hopperextend = new TalonFX(k_hopperextendID);
  private final TalonFX m_rumblePak = new TalonFX(k_rumblePakID);

  /** Hopper extension subsystem. */
  public Hopper() {
    m_hopperextend.getConfigurator().apply(k_hopperextendconfig);
    m_rumblePak.getConfigurator().apply(k_rumblepakconfig);
  }

  /* Starts the rumblePak */
  public void agitate() {
    m_rumblePak.setVoltage(9);
  }

  /* Stops the rumblePak */
  public void stopAgitate() {
    m_rumblePak.stopMotor();
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
  public void stopHopper() {
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
