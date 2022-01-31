// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.List;

import org.team2168.commands.*;
import org.team2168.commands.drivetrain.*;
import org.team2168.commands.turret.*;
import org.team2168.commands.exampleSubsystem.*;
import org.team2168.commands.monkeybar.*;
import org.team2168.commands.climber.*;
import org.team2168.commands.pixy.*;
import org.team2168.subsystems.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Pixy m_pixy = Pixy.getInstance();

  public final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Turret m_turret = Turret.getInstance();
  private final MonkeyBar monkeyBar = MonkeyBar.getInstance();

  private ExtendExample extendExampleSubsystem= new ExtendExample(m_exampleSubsystem);
  private RetractExample retractExampleSubsystem= new RetractExample(m_exampleSubsystem);
  private final FindAllianceBall m_findAllianceBall = new FindAllianceBall(m_pixy);

  OI oi = OI.getInstance();

  private static RobotContainer instance = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);

    m_pixy.setDefaultCommand(m_findAllianceBall);
    
    // Configure the button bindings
    configureButtonBindings();
  }

  public static RobotContainer getInstance() {
    if (instance == null)
      instance = new RobotContainer();
    return instance;
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    //Driver Controls
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));

    //Operator Controls
    m_turret.setDefaultCommand(new DriveTurretWithJoystick(m_turret, oi.operatorJoystick::getLeftStickRaw_X));
    climber.setDefaultCommand(new DriveClimberWithJoystick(climber, oi.operatorJoystick::getLeftStickRaw_Y));

    oi.operatorJoystick.ButtonA().whenPressed(new ExtendMonkeyBar(monkeyBar));
    oi.operatorJoystick.ButtonA().whenReleased(new RetractMonkeyBar(monkeyBar));
    oi.operatorJoystick.ButtonX().whenHeld(new SetPosition(climber, 12.0));
    oi.operatorJoystick.ButtonY().whenPressed(new ReturnToZero(climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // I don't see a point of having radian conversions in our actual code, we won't
    // // need them after characterization
    // DoubleFunction<Double> degToRadians = (d) -> d * (Math.PI / 180.0);
    // DoubleFunction<Double> ticksToRadians = (t) -> ((t / Drivetrain.TICKS_PER_REV) / Drivetrain.GEAR_RATIO) * 2.0
    //     * Math.PI;

    // return new SysIDCommand(
    //     drivetrain, (l, r) -> drivetrain.tankDriveVolts(l, r),
    //     () -> {
    //       return new SysIDCommand.DriveTrainSysIdData(
    //           ticksToRadians.apply(drivetrain.getLeftEncoderDistanceRaw()),
    //           ticksToRadians.apply(drivetrain.getRightEncoderDistanceRaw()),
    //           ticksToRadians.apply(drivetrain.getLeftEncoderRateRaw()),
    //           ticksToRadians.apply(drivetrain.getRightEncoderRateRaw()),
    //           degToRadians.apply(drivetrain.getHeading()),
    //           degToRadians.apply(drivetrain.getTurnRate()));
    //     }); // Drivetrain characterization

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.Drivetrain.ksVolts,
                Constants.Drivetrain.kvVoltSecondsPerMeter,
                Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
                Constants.Drivetrain.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.Drivetrain.kMaxSpeedMetersPerSecond,
                Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Drivetrain.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(3.31, 0.44, new Rotation2d(0))),
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(Constants.Drivetrain.kRamseteB, Constants.Drivetrain.kRamseteZeta),
            new SimpleMotorFeedforward(
              Constants.Drivetrain.ksVolts,
              Constants.Drivetrain.kvVoltSecondsPerMeter,
              Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
              Constants.Drivetrain.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
            new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  @Config(rowIndex = 3, columnIndex = 0, width = 1, height = 1, tabName = "ExampleSubsystem")
  private void retractExample(boolean foo) {
    retractExampleSubsystem.schedule();
  }

  @Config(rowIndex = 3, columnIndex = 1, width = 1, height = 1, tabName = "ExampleSubsystem")
  private void extendExample(boolean foo) {
    extendExampleSubsystem.schedule();
  }
}
