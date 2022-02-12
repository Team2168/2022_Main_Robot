// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.List;
import java.util.function.DoubleFunction;

import org.team2168.commands.SysIDCommand;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.auto.Drive1Meter;
import org.team2168.commands.auto.Drive3Meters;
import org.team2168.commands.auto.FourBall;
import org.team2168.commands.auto.MultipartNonZeroVel;
import org.team2168.commands.auto.Paths;
import org.team2168.commands.auto.TwoballTopToTerm;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.drivetrain.ResetHeading;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

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
  // private final Pixy m_pixy = Pixy.getInstance();

  public final Drivetrain drivetrain = Drivetrain.getInstance();
  // private final Climber climber = Climber.getInstance();
  // private final Turret m_turret = Turret.getInstance();
  // private final MonkeyBar monkeyBar = MonkeyBar.getInstance();

  // private ExtendExample extendExampleSubsystem= new ExtendExample(m_exampleSubsystem);
  // private RetractExample retractExampleSubsystem= new RetractExample(m_exampleSubsystem);
  // private final FindAllianceBall m_findAllianceBall = new FindAllianceBall(m_pixy);

  OI oi = OI.getInstance();

  @Log(name = "Auto Chooser", width = 2)
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private boolean brakesEnabled = true;
  private Paths paths;

  private static RobotContainer instance = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    paths = Paths.getInstance();
    Logger.configureLoggingAndConfig(this, false);

    // m_pixy.setDefaultCommand(m_findAllianceBall);
    
    // Configure the button bindings
    configureButtonBindings();
    configureAutonomousRoutines();
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

    oi.testJoystick.ButtonStart().whenPressed(new ResetHeading(drivetrain));

    //Operator Controls
    // m_turret.setDefaultCommand(new DriveTurretWithJoystick(m_turret, oi.operatorJoystick::getLeftStickRaw_X));
    // climber.setDefaultCommand(new DriveClimberWithJoystick(climber, oi.operatorJoystick::getLeftStickRaw_Y));

    // oi.operatorJoystick.ButtonA().whenPressed(new ExtendMonkeyBar(monkeyBar));
    // oi.operatorJoystick.ButtonA().whenReleased(new RetractMonkeyBar(monkeyBar));
    // oi.operatorJoystick.ButtonX().whenHeld(new SetPosition(climber, 12.0));
    // oi.operatorJoystick.ButtonY().whenPressed(new ReturnToZero(climber));
  }

  private void configureAutonomousRoutines() {
    autoChooser.setDefaultOption("Do nothing", new DoNothing());
    autoChooser.addOption("2 Ball Top to Terminal", new TwoballTopToTerm(drivetrain));
    autoChooser.addOption("4 Ball (ends at Terminal)", new FourBall(drivetrain));

    // debug autos
    autoChooser.addOption("Drive 1 Meter", new Drive1Meter(drivetrain));
    autoChooser.addOption("Drive 3 Meters", new Drive3Meters(drivetrain));
    autoChooser.addOption("multipart nonzero start/end velocity", new MultipartNonZeroVel(drivetrain));
    autoChooser.addOption("Test Trajectory Command", getExampleTrajectoryCommand());
    // autoChooser.addOption("Debug auto", new DebugPathWeaver(drivetrain, "Drive3Meters"));
    // autoChooser.addOption("Squiggles", new Squiggles(drivetrain));
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var auto = autoChooser.getSelected();
    if (auto == null) {
      System.out.println("Selected path is null!");
      return new DoNothing();
    }
    return autoChooser.getSelected();
  }

  
  public Command getCharacterizationCommand() {
    // I don't see a point of having radian conversions in our actual code, we won't
    // need them after characterization
    DoubleFunction<Double> degToRadians = (d) -> d * (Math.PI / 180.0);
    DoubleFunction<Double> ticksToRadians = (t) -> ((t / Drivetrain.TICKS_PER_REV) / Drivetrain.GEAR_RATIO) * 2.0
        * Math.PI;

    return new SysIDCommand(
        drivetrain, (l, r) -> drivetrain.tankDriveVolts(l, r),
        () -> {
          return new SysIDCommand.DriveTrainSysIdData(
              ticksToRadians.apply(drivetrain.getLeftEncoderDistanceRaw()),
              ticksToRadians.apply(drivetrain.getRightEncoderDistanceRaw()),
              ticksToRadians.apply(drivetrain.getLeftEncoderRateRaw()),
              ticksToRadians.apply(drivetrain.getRightEncoderRateRaw()),
              degToRadians.apply(drivetrain.getHeading()),
              degToRadians.apply(drivetrain.getTurnRate()));
        }); // Drivetrain characterization
  }

  @Config(rowIndex = 3, columnIndex = 0, width = 1, height = 1, tabName = "ExampleSubsystem")
  private void retractExample(boolean foo) {
    // retractExampleSubsystem.schedule();
  }

  @Config(rowIndex = 3, columnIndex = 1, width = 1, height = 1, tabName = "ExampleSubsystem")
  private void extendExample(boolean foo) {
    // extendExampleSubsystem.schedule();
  }

  public boolean brakesEnabled() {
    return brakesEnabled;
  }

  @Config(name = "Kickable Robot?", width = 2)
  public void setBrakesEnabled(boolean enabled) {
    brakesEnabled = enabled;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getExampleTrajectoryCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              Constants.Drivetrain.ksVolts,
              Constants.Drivetrain.kvVoltSecondsPerMeter,
              Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
              Constants.Drivetrain.kDriveKinematics,
            Constants.Drivetrain.MAX_VOLTAGE);

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
    Trajectory path1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(1, 0, new Rotation2d(0))),
            config);

    Trajectory path2 = 
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(0, -5.67, new Rotation2d(Units.degreesToRadians(90.0)))),
            config);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(path1.getInitialPose());

    // Run path following command, then stop at the end.
    return makeRamsete(path1)
      .andThen(() -> drivetrain.tankDriveVolts(0, 0))
      .andThen(makeRamsete(path2))
      .andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  RamseteCommand makeRamsete(Trajectory path) {
    return new RamseteCommand(
          path,
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
  }
}