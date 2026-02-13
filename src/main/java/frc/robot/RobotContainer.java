// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.ClimberConstants.k_highlimit;
import static frc.robot.utils.Constants.ShooterConstants.k_shootermaxvel;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.LLVision;
import frc.robot.subsystems.mechanisms.Climber;
import frc.robot.subsystems.mechanisms.Feeder;
import frc.robot.subsystems.mechanisms.Hopper;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.Shooter;
import frc.robot.subsystems.swerve.CTRESwerve;
import frc.robot.subsystems.swerve.SwerveTelemetry;
import frc.robot.subsystems.swerve.SwerveTunerConstants;
import frc.robot.utils.Constants.ClimberConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {

  // joystick
  private final CommandGenericHID m_driverctlr = new CommandGenericHID(0);

  // other subsystems
  private final SwerveTelemetry m_swervetelemetry = new SwerveTelemetry(SwerveConstants.k_maxlinspeed);
  private final CTRESwerve m_swerve = SwerveTunerConstants.createDrivetrain();
  private final Climber m_climber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder();
  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();
  public final LLVision m_limelight = new LLVision();

  // autochooser
  private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

  // commands
  private final Command m_hopperextend = runEnd(() -> m_hopper.extend(), () -> m_hopper.stop(), m_hopper).withTimeout(1.2);
  private final Command m_hopperretract = runEnd(() -> m_hopper.retract(), () -> m_hopper.stop(), m_hopper).withTimeout(1.2);
  private final Command m_autofeed = parallel(
    runEnd(() -> m_feeder.feed(), () -> m_feeder.stop(), m_feeder),
    runEnd(() -> m_hopper.retract(), () -> m_hopper.stop(), m_hopper)
  );
  private final Command m_manualfeed = runEnd(() -> m_feeder.feed(), () -> m_feeder.stop(), m_feeder);
  private final Command m_intakefloor = runEnd(() -> m_intake.intake(), () -> m_intake.stop(), m_intake);
  private final Command m_climberup = 
    runEnd(() -> m_climber.climberManual(6), () -> m_climber.climberStop(), m_climber)
    .until(() -> m_climber.averageDistance() > ClimberConstants.k_highlimit);
  private final Command m_climberdown =
    runEnd(() -> m_climber.climberManual(-6), () -> m_climber.climberStop(), m_climber)
    .until(() -> m_climber.averageDistance() < 0);
  private final Command m_climb = 
      runEnd(() -> m_climber.climberManual(-12), () -> m_climber.climberStop(), m_climber)
      .until(() -> m_climber.averageDistance() < k_highlimit / 2);

  private final Command driveAndAim = m_swerve.applyRequest(() -> new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withVelocityX(MathUtil.applyDeadband((m_limelight.aimAndRange()[0]), OIConstants.k_limelightDeadZone) * SwerveConstants.k_maxlinspeed * 0.5) // x velocity
        .withVelocityY(MathUtil.applyDeadband(m_driverctlr.getRawAxis(0), OIConstants.k_deadzone) * SwerveConstants.k_maxlinspeed * 0.5) // y velocity
        .withRotationalRate(MathUtil.applyDeadband((-m_limelight.aimAndRange()[1]), OIConstants.k_limelightDeadZone) * SwerveConstants.k_maxrotspeed) // z rot velocity
    );

  // misc vars
  private double shootervel = 0;

  public RobotContainer() {
    // pathfinding algo
    Pathfinding.setPathfinder(new LocalADStar());
    m_swerve.configureAutobuilder();
    // add autos
    // m_autochooser.setDefaultOption("Pathfind 1", m_swerve.pathfind(new Pose2d(2, 2, new Rotation2d())));
    SmartDashboard.putData(m_autochooser);
    // default commands
    m_shooter.setDefaultCommand(shooterDefault());
    m_swerve.setDefaultCommand(swerveDefault());
    // start telemtry for swerve
    m_swerve.registerTelemetry(m_swervetelemetry::telemeterize);
    // configure triggers
    configureBindings();
  }

  private void configureBindings() {
    // idles drivetrain when disabled
    RobotModeTriggers.disabled().whileTrue(
        m_swerve.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));
    // drivetrain bindings
    m_driverctlr.button(6).whileTrue(
      m_swerve.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
    );
    // shooter velocity setpoint
    m_driverctlr.button(8).onTrue(runOnce(() -> shootervel = k_shootermaxvel));
    m_driverctlr.button(7).onTrue(runOnce(() -> shootervel = 0));
    // feeder controls
    m_driverctlr.button(2).whileTrue(m_autofeed);
    m_driverctlr.button(3).whileTrue(m_manualfeed);
    // hopper controls
    m_driverctlr.povLeft().onTrue(m_hopperextend);
    m_driverctlr.povRight().onTrue(m_hopperretract);
    // climber
    m_driverctlr.povUp().onTrue(m_climberup);
    m_driverctlr.povDown().onTrue(m_climberdown);
    m_driverctlr.button(4).onTrue(m_climb);
    // intake
    m_driverctlr.button(1).toggleOnTrue(m_intakefloor);
    // aim
    new Trigger(() -> Math.abs(m_driverctlr.getRawAxis(2)) > 0.5)
    .whileTrue(driveAndAim);
  }

  /** Default command for the shooter subsytem. */
  public Command shooterDefault() {
    return run(() -> m_shooter.closedLoop(shootervel), m_shooter);
  }

  /** Default operator controlled method for swerve subsystem. */
  public Command swerveDefault() {
    return m_swerve.applyRequest(() -> new SwerveRequest.FieldCentric()
        .withRotationalDeadband(SwerveConstants.k_maxrotspeed * OIConstants.k_deadzone) // 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withVelocityX(MathUtil.applyDeadband(m_driverctlr.getRawAxis(1), OIConstants.k_deadzone) * SwerveConstants.k_maxlinspeed * 0.5) // x velocity
        .withVelocityY(MathUtil.applyDeadband(m_driverctlr.getRawAxis(0), OIConstants.k_deadzone) * SwerveConstants.k_maxlinspeed * 0.5) // y velocity
        .withRotationalRate(-MathUtil.applyDeadband(m_driverctlr.getRawAxis(4), OIConstants.k_deadzone) * SwerveConstants.k_maxrotspeed) // z rot velocity
    );
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.pathfindToPose(new Pose2d(1, 1, new Rotation2d()), PathConstraints.unlimitedConstraints(12));
  }
}
