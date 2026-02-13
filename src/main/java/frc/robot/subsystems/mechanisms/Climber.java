// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.utils.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

  // motor instances
  private final TalonFX m_leftclimber = new TalonFX(k_leftclimberID);
  private final TalonFX m_rightclimber = new TalonFX(k_rightclimberID);

  // climber PIDF
  private final ProfiledPIDController m_climbPID = new ProfiledPIDController(
      k_climberkP,
      k_climberkI,
      k_climberkD,
      k_climberConstraints);
  private final ElevatorFeedforward m_climbFF = new ElevatorFeedforward(
      k_climberkS,
      k_climberkG,
      k_climberkV);

  // persistent position values
  private double leftpos = 0;
  private double rightpos = 0;

  /** Climber subsystem. Includes deploying and climbing. */
  public Climber() {
    // apply configs
    m_leftclimber.getConfigurator().apply(k_climberconfig);
    m_rightclimber.getConfigurator().apply(k_climberconfig);

    // reset positions
    m_leftclimber.setPosition(0);
    m_rightclimber.setPosition(0);
  }

  /** Retracts the climber with PID control. */
  public void climberRetractClosedLoop(double setpoint) {
    m_climbPID.setGoal(setpoint);
    double pid = m_climbPID.calculate((leftpos + rightpos) / 2);
    double ff = m_climbFF.calculate(m_climbPID.getSetpoint().velocity);
    m_leftclimber.setVoltage(ff + pid);
    m_rightclimber.setVoltage(ff + pid);
  }

  /** Retracts the climber using a FF only. */
  public void climberRetractOpenLoop(double velocitysp) {
    double ff = m_climbFF.calculate(velocitysp);
    m_leftclimber.setVoltage(ff);
    m_rightclimber.setVoltage(ff);
  }

  public void climberManual(double volts) {
    m_leftclimber.setVoltage(volts);
    m_rightclimber.setVoltage(volts);
  }

  public void climberStop() {
    m_leftclimber.stopMotor();
    m_rightclimber.stopMotor();
  }

  public void resetEncoders() {
    m_leftclimber.setPosition(0);
    m_rightclimber.setPosition(0);
  }

  public double averageDistance() {
    return (leftpos + rightpos) / 2;
  }

  /** Returns a pair of doubles containing the positions. Left is first, right is second. */
  public Pair<Double,Double> climberPositions() {
    return Pair.of(leftpos, rightpos);
  }

  @Override
  public void periodic() {
    // update positions periodically
    leftpos = (m_leftclimber.getPosition().getValue().in(Units.Rotations) / k_climberratio) * k_climberwinchcirc;
    rightpos = (m_rightclimber.getPosition().getValue().in(Units.Rotations) / k_climberratio) * k_climberwinchcirc;

    // positions
    SmartDashboard.putNumber("leftpos", leftpos);
    SmartDashboard.putNumber("rightpose", rightpos);
  }
}
