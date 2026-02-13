// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static frc.robot.utils.Constants.FeederConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  // motor instances
  private final TalonFX m_shooterfeed1 = new TalonFX(k_shooterfeed1ID);

  public Feeder() {

    // apply configs
    m_shooterfeed1.getConfigurator().apply(k_shooterfeedconfig);
  }

  /** Runs the feeder roller. */
  public void feed() {
    m_shooterfeed1.setVoltage(12);
  }

  /** Stops the feeder roller. */
  public void stop() {
    m_shooterfeed1.setVoltage(0);
  }

  @Override
  public void periodic() {
  }
}
