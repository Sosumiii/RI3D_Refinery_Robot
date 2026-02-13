// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  // motor instances
  private final TalonFX m_shooter1 = new TalonFX(k_shooter1ID);

  // ff and pid controller, need to characterize
  private final ProfiledPIDController m_velocityPID = new ProfiledPIDController(
    k_velocitykP,
    k_velocitykI, 
    k_velocitykD, 
    k_velocityConstraints);
  private final SimpleMotorFeedforward m_velocityFF = new SimpleMotorFeedforward(
    k_velocitykS, 
    k_velocitykV);

  // misc vars
  private double velocityMPS = 0;

  /** Shooter subsystem. Duh. */
  public Shooter() {

    // apply the built config to the motor
    m_shooter1.getConfigurator().apply(k_shooterconfig);
  }

  /** Closed loop control using PIDF. */
  public void closedLoop(double velocitysp) {
    m_velocityPID.setGoal(velocitysp);
    double pid = m_velocityPID.calculate(velocityMPS);
    double ff = m_velocityFF.calculate(velocitysp);
    m_shooter1.setVoltage(ff + pid);
    SmartDashboard.putNumber("output", ff + pid);
  }

  /** Open loop control using only the FF */
  public void openLoop(double velocitysp) {
    double ff = m_velocityFF.calculate(velocitysp);
    m_shooter1.setVoltage(ff);
  }

  /** Stops the shooter motor and resets the PID controller to zero.*/
  public void stopShooter() {
    m_shooter1.stopMotor();
    m_velocityPID.reset(0);
  }

  public double getVelocity() {
    return velocityMPS;
  }

  @Override
  public void periodic() {
    velocityMPS = ((m_shooter1.getVelocity().getValue().in(Units.RPM) / k_shooterratio) * k_shooterwheelcircum) / 60; //returns in units of m/s
  }
}
