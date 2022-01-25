// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.List;
import java.util.function.DoubleFunction;

import org.team2168.commands.SysIDCommand;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import io.github.oblarg.oblog.Logger;

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
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Drivetrain drivetrain = Drivetrain.getInstance();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  OI oi = OI.getInstance();

  private static RobotContainer instance = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);

    // Configure the button bindings
    configureButtonBindings();
  }

  public static RobotContainer getInstance() {
    if (instance == null)
      instance = new RobotContainer();
    return instance;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getDriverJoystickX, oi::getDriverJoystickY));
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
}
