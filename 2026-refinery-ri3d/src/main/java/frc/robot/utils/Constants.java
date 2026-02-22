// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import frc.robot.subsystems.swerve.SwerveTunerConstants;

public class Constants {

  public class OIConstants {
    public static final double k_deadzone = 0.2;
    public static final double k_limelightDeadZone = 0.06;
    public static final double k_maxlinspeedteleop = 3;
    public static final double k_maxrotspeedteleop = 2 * Math.PI;
  }

  public class SwerveConstants {
    public static final double k_maxlinspeed = 1.0 * SwerveTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double k_maxrotspeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  }

  public class ShooterConstants {
    // motor configuration
    public static final int k_shooter1ID = 18;
    public static final TalonFXConfiguration k_shooterconfig = new TalonFXConfiguration();
    static {
      k_shooterconfig.CurrentLimits.StatorCurrentLimit = 100;
      k_shooterconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_shooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_shooterconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    // misc shooter constants
    public static final double k_shooterwheeldiam = Units.inchesToMeters(4); //meters
    public static final double k_shooterwheelcircum = Math.PI * k_shooterwheeldiam; //meters
    public static final double k_shooterratio = 0.588;
    public static final double k_shootermotormaxspeed = 4600.0; //rpm
    public static final double k_shootermaxvel = ((k_shootermotormaxspeed / k_shooterratio) * k_shooterwheelcircum) / 60; //m/s
    // PIDF constants, needs characterized
    public static final double k_velocitykP = 0.5;
    public static final double k_velocitykI = 0;
    public static final double k_velocitykD = 0;
    public static final double k_velocitykS = 0; // characterize pls
    public static final double k_velocitykV = 12 / k_shootermaxvel;
    public static final TrapezoidProfile.Constraints k_velocityConstraints =
        new TrapezoidProfile.Constraints(
            k_shootermotormaxspeed,
            k_shootermotormaxspeed);
  }

  public class FeederConstants {
    public static final int k_shooterfeed1ID = 20;
    public static final TalonFXConfiguration k_shooterfeedconfig = new TalonFXConfiguration();
    static {
      k_shooterfeedconfig.CurrentLimits.StatorCurrentLimit = 60;
      k_shooterfeedconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_shooterfeedconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_shooterfeedconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
  }

  public class IntakeConstants {
    // motor configuration
    public static final int k_intakerollerID = 22;
    public static final TalonFXConfiguration k_intakerollerconfig = new TalonFXConfiguration();
    static {
      k_intakerollerconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_intakerollerconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_intakerollerconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_intakerollerconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
  }

  public class BootyConstants {
    // motor configuration
    public static final int k_hopperextendID = 23;
    public static final int k_rumblePakID = 26;

    public static final TalonFXConfiguration k_rumblepakconfig = new TalonFXConfiguration();
    static {
      k_rumblepakconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_rumblepakconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_rumblepakconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_rumblepakconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    public static final TalonFXConfiguration k_hopperextendconfig = new TalonFXConfiguration();
    static {
      k_hopperextendconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_hopperextendconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_hopperextendconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_hopperextendconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
  }

  public class ClimberConstants {
    // climber lift motor configuration
    public static final int k_leftclimberID = 24;
    public static final int k_rightclimberID = 25;
    public static final TalonFXConfiguration k_climberconfig = new TalonFXConfiguration();
    static {
      k_climberconfig.CurrentLimits.StatorCurrentLimit = 60;
      k_climberconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_climberconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      k_climberconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    public static final double k_highlimit = 0.22; // meters
    public static final double k_lowlimit = 0; // meters
    public static final double k_positiontolerance = Units.inchesToMeters(1); //meters
    public static final double k_climberratio = 20; 
    public static final double k_climberwinchdiam = Units.inchesToMeters(1); //meters
    public static final double k_climberwinchcirc = k_climberwinchdiam * Math.PI; //meters
    public static final double k_climbermotormaxspeed = 6000; //rpm, currently kraken x60
    public static final double k_climbermaxvel = ((k_climbermotormaxspeed / k_climberratio) * k_climberwinchcirc) / 60; //m/s
    // PIDF for climbing
    public static final double k_climberkP = 0.1;
    public static final double k_climberkI = 0;
    public static final double k_climberkD = 0;
    public static final double k_climberkS = 0.1;
    public static final double k_climberkG = 2;
    public static final double k_climberkV = 12 / k_climbermaxvel;
    public static final TrapezoidProfile.Constraints k_climberConstraints = 
        new TrapezoidProfile.Constraints(
            k_climbermaxvel, //m/s
            k_climbermaxvel); //m/s^2
  }

  public static final class PathfindingConstants {

  }
}
