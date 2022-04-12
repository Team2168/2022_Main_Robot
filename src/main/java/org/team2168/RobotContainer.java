// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import org.team2168.commands.*;
import org.team2168.commands.LEDs.ShowShooterAtSpeed;
import org.team2168.commands.auto.*;
import org.team2168.commands.auto.pathplanner.FiveBall;
import org.team2168.commands.auto.pathplanner.FourBall;
// import org.team2168.commands.auto.pathplanner.DebugPathPlanner;
import org.team2168.commands.auto.pathplanner.ThreeBall;
import org.team2168.commands.auto.pathplanner.TwoBall;
import org.team2168.commands.climber.DriveClimber;
import org.team2168.commands.climber.FullSendClimbingSequence;
import org.team2168.commands.drivetrain.*;
import org.team2168.commands.hood.BumpHoodAngleDown;
import org.team2168.commands.hood.BumpHoodAngleUp;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.hopper.DriveHopperUntilBall;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.indexer.DriveIndexerUntilBall;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.limelight.SetPipeline;
import org.team2168.commands.monkeybar.*;
import org.team2168.commands.pooper.PooperUnpoop;
import org.team2168.commands.shooter.BumpShooterSpeedDown;
import org.team2168.commands.shooter.BumpShooterSpeedUp;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.commands.shootingpositions.*;
import org.team2168.commands.turret.*;
import org.team2168.subsystems.*;
import org.team2168.subsystems.Shooter.ShooterRPM;

import java.util.List;
import java.util.function.DoubleFunction;

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
  public final Drivetrain drivetrain = Drivetrain.getInstance();
  public final Hopper hopper = Hopper.getInstance();
  public final Pooper pooper = Pooper.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();
  // private final Turret turret = Turret.getInstance(); // motor not powered for time being
  private final MonkeyBar monkeyBar = MonkeyBar.getInstance();
  public final Limelight lime = Limelight.getInstance();
  public final IntakeRoller intakeRoller = IntakeRoller.getInstance();
  private final Indexer indexer = Indexer.getInstance();
  private final Hood hood = Hood.getInstance();
  private final ColorSensor colorSensor = ColorSensor.getInstance();
  private final Turret turret = Turret.getInstance();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  private final IntakeRaiseAndLower intakeRaiseAndLower= IntakeRaiseAndLower.getInstance();
  private final LEDs leds = LEDs.getInstance();

  OI oi = OI.getInstance();

  @Log(name = "Auto Chooser", width = 2)
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private boolean brakesEnabled = false;

  private static RobotContainer instance = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // Paths.getInstance();  // Create the instance so paths are generated soon after code execution begins, well before autos
    Logger.configureLoggingAndConfig(this, false);

    if (Constants.IS_COMPBOT) {
      System.out.println("DID NOT FIND PRACTICE BOT JUMPER -- USING COMPETITION BOT GAINS");
    } else {
      System.out.println("FOUND PRACTICE BOT JUMPER -- CONFIGURING WITH PBOT GAINS");
    }
    // Configure the button bindings
    configureButtonBindings();
    configureAutonomousRoutines();
  }

  public static RobotContainer getInstance() {
    if (instance == null)
      instance = new RobotContainer();
    return instance;
  }

  private void configureAutonomousRoutines() {
    var twoBall = new SequentialCommandGroup(
      new Sleep().withTimeout(0.0),
      new TwoBall(
              drivetrain, intakeRaiseAndLower, intakeRoller,
              hopper, indexer, hood,
              shooter, turret, pooper, colorSensor,
              lime)
    );

    autoChooser.setDefaultOption("Do nothing", new DoNothing());
    autoChooser.addOption("2 ball", twoBall);
      autoChooser.addOption("3 Ball", new ThreeBall(
              drivetrain, intakeRaiseAndLower, intakeRoller,
              hopper, indexer, hood,
              shooter, turret, pooper, colorSensor,
              lime));
    autoChooser.addOption("4 ball", new FourBall(
      drivetrain, intakeRaiseAndLower, intakeRoller, hopper, 
      indexer, turret, hood, shooter, lime, pooper, colorSensor));

    autoChooser.addOption("5 Ball", new FiveBall(
      drivetrain, intakeRaiseAndLower, intakeRoller,
      hopper, indexer, hood,
      shooter, turret, pooper, colorSensor,
      lime));

    // autoChooser.addOption("TestTurn", new TurnXDegrees(drivetrain, 92.0));
    // autoChooser.addOption("TestDriveStraight", new DriveXDistance(drivetrain, 8.0 * 12.0));
    // autoChooser.addOption("test drive", new DebugPathPlanner(drivetrain, "Drive3Meters"));
    //     autoChooser.addOption("2 Ball Top to Terminal", new TwoballTopToTerm(drivetrain));
    //   autoChooser.addOption("OppositeSideToTerminal", new SimpleOppositeSideToTerminal(
    //           drivetrain, intakeRaiseAndLower, intakeRoller,
    //           hopper, indexer, hood,
    //           shooter, pooper, colorSensor,
    //           lime));
    // autoChooser.addOption(
    //         "4 Ball (ends at Terminal)", new FourBall(
    //                     drivetrain, intakeRaiseAndLower, intakeRoller,
    //                     hopper, indexer, hood,
    //                     shooter, pooper, colorSensor,
    //                     lime));
    // autoChooser.addOption("Drive 3 Feet",
    //         new DriveXDistance(drivetrain, Units.metersToInches(1.0)));
    // autoChooser.addOption("Drive 3 Feet",
    //         new DriveXDistance(drivetrain, Units.metersToInches(-1.0)));
    // autoChooser.addOption("rotate35", new TurnXDegrees(drivetrain, 35.0));
    // autoChooser.addOption("rotate-35", new TurnXDegrees(drivetrain, -35.0));

    // debug autos
    // autoChooser.addOption("Drive 1 Meter", new Drive1Meter(drivetrain));
    // autoChooser.addOption("Drive 3 Meters", new Drive3Meters(drivetrain));
    // autoChooser.addOption("Debug drive 1 meter", new DebugPathPlanner(drivetrain, "Drive1Meter"));
    // autoChooser.addOption("Test Trajectory Command", getExampleTrajectoryCommand());
    // autoChooser.addOption("Debug auto", new DebugPathWeaver(drivetrain, "Drive3Meters"));
    // autoChooser.addOption("Squiggles", new Squiggles(drivetrain));
  
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    leds.setDefaultCommand(new ShowShooterAtSpeed(leds, shooter, climber));

    //DRIVER CONTROLS
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));

    //// Green button
    oi.driverJoystick.ButtonLeftStick()
            .whenHeld(new DriveWithLimelight(drivetrain, lime, oi::getGunStyleTrigger))
            .whenPressed(new RotateTurret(turret, 0.0));
    

    //// Black button
    oi.driverJoystick.ButtonRightBumper()
            .whenPressed(new HoodToAngle(hood, 0.0).withTimeout(0.5))
            .whenPressed(new SetShooterSpeed(shooter, ShooterRPM.STOP).withTimeout(0.2))
            .whenPressed(new RotateTurret(turret, -90.0).withTimeout(0.5)
                .andThen(new StopTurret(turret))
                .andThen(new FullSendClimbingSequence(climber, monkeyBar))
            );

    //lower left button ("Forward Fine-Tuning")
    oi.driverJoystick.ButtonA().whenPressed(new StowEverything(hood, shooter));

    //red button
    oi.driverJoystick.ButtonLeftBumper().whenPressed(new DriveTurretWithLimelight(turret, lime));

    //OPERATOR CONTROLS
    //// main button cluster
    // oi.operatorJoystick.ButtonA()
    //         .whenPressed(new RotateTurret(turret, 0.0))
    //         .whenPressed(new FenderLow(hood, shooter));

    oi.operatorJoystick.ButtonA().whenPressed(new FenderLow(hood, shooter));
    oi.operatorJoystick.ButtonB().whenPressed(new TarmacLine(hood, shooter, lime));
    oi.operatorJoystick.ButtonX().whenPressed(new Launchpad(hood, shooter, lime));
    oi.operatorJoystick.ButtonY()
            .whenPressed(new RotateTurret(turret, 0.0))
            .whenPressed(new FenderHigh(hood, shooter, lime));

    //// start and back
    oi.operatorJoystick.ButtonStart().whenPressed(new BumpShooterSpeedUp(shooter));
    oi.operatorJoystick.ButtonBack().whenPressed(new BumpShooterSpeedDown(shooter));

    //// dpad
    // oi.operatorJoystick.ButtonUpDPad().whenPressed(new ManuallyStageBall(indexer)); // TODO implement manual staging
    oi.operatorJoystick.ButtonUpDPad().whenPressed(new DriveIndexerUntilBall(indexer, () -> Constants.MotorSpeeds.INDEXER_SPEED));
    oi.operatorJoystick.ButtonDownDPad().whenPressed(new DriveHopperUntilBall(hopper, () -> Constants.MotorSpeeds.HOPPER_SPEED));
    // oi.operatorJoystick.ButtonDownDPad().whenPressed(new DriveClimberToZero(climber));
    oi.operatorJoystick.ButtonLeftDPad().whenPressed(new BumpHoodAngleDown(hood));
    oi.operatorJoystick.ButtonRightDPad().whenPressed(new BumpHoodAngleUp(hood));

    //// sticks
    oi.operatorJoystick.ButtonLeftStick().whenPressed(new DriveClimber(climber, oi.operatorJoystick::getLeftStickRaw_Y));

    oi.operatorJoystick.ButtonRightStickMovedLeft().and(oi.operatorJoystick.ButtonRightStick()).whenActive(new ExtendMonkeyBar(monkeyBar));
    oi.operatorJoystick.ButtonRightStickMovedRight().and(oi.operatorJoystick.ButtonRightStick()).whenActive(new RetractMonkeyBar(monkeyBar));

    oi.operatorJoystick.ButtonRightStickMovedUp()
      .whenPressed(new RotateTurret(turret, 0.0).andThen(new DriveTurretWithLimelight(turret, lime)));

    //// Trigger cluster
    oi.operatorJoystick.ButtonLeftBumper()
            // .whileHeld(new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller))
            .whileHeld(new QueueBallForShot(hopper, indexer, pooper, colorSensor, intakeRoller))
            .whenPressed(new IntakeLower(intakeRaiseAndLower))
            .whenReleased(new IntakeRaise(intakeRaiseAndLower))
            .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
            .whenReleased(new SetIntakeSpeed(intakeRoller, 0.0))
            .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)))
            .whenReleased(new PooperUnpoop(pooper));

    oi.operatorJoystick.ButtonRightBumper()
            .whileHeld(new FireBalls(shooter, indexer, hopper))
            //.whenPressed(new FireBalls(shooter, indexer, hopper))
            //.whenPressed(new DriveHopperAndIndexer(hopper, indexer))
            .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
            .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)));

    oi.operatorJoystick.ButtonLeftTrigger()
            .whenPressed(new HoodToAngle(hood, 0.0))
            .whenPressed(new SetShooterSpeed(shooter, ShooterRPM.STOP));

    oi.operatorJoystick.ButtonRightTrigger()
            .whenPressed(new ShootBasedOnDistance(shooter, hood, lime, oi));


    //TEST JOYSTICK
    // oi.testJoystick.ButtonLeftBumper()

    //         // .whileHeld(new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller))
    //         .whileHeld(new QueueBallForShot(hopper, indexer, pooper, colorSensor, intakeRoller))
    //         .whenPressed(new IntakeLower(intakeRaiseAndLower))
    //         .whenReleased(new IntakeRaise(intakeRaiseAndLower))
    //         .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
    //         .whenReleased(new SetIntakeSpeed(intakeRoller, 0.0))
    //         .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)))
    //         .whenReleased(new PooperUnpoop(pooper));

    // oi.testJoystick.ButtonRightBumper()
    //         .whileHeld(new FireBalls(shooter, indexer, hopper))
    //         //.whenPressed(new FireBalls(shooter, indexer, hopper))
    //         //.whenPressed(new DriveHopperAndIndexer(hopper, indexer))
    //         .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
    //         .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)));

    // oi.testJoystick.ButtonY().whenPressed(new BumpHoodAngleZero(hood));
    
    // oi.testJoystick.ButtonX().whenPressed(new BumpHoodAngleDown(hood));
    // oi.testJoystick.ButtonB().whenPressed(new BumpHoodAngleUp(hood));

    // oi.testJoystick.ButtonLeftDPad().whenPressed(new BumpShooterSpeedDown(shooter));
    // oi.testJoystick.ButtonRightDPad().whenPressed(new BumpShooterSpeedUp(shooter));

    // oi.testJoystick.ButtonA().whenPressed(new HoodToAngle(hood, HoodPosition.AUTO_TARMAC_LINE.position_degrees))
    //                          .whenPressed(new SetShooterSpeed(shooter, ShooterRPM.AUTO_SIMPLE_4_BALL));
    // oi.testJoystick.ButtonA().whenPressed(new RotateTurret(turret, 0.0));
    // oi.testJoystick.ButtonB().whenPressed(new RotateTurret(turret, 180));
    // oi.testJoystick.ButtonX().whenPressed(new RotateTurret(turret, -180));
    // turret.setDefaultCommand(new DriveTurretWithJoystick(turret, oi::getTestJoystickX));

    // oi.testJoystick.ButtonStart().whenPressed(new FindTargetWithTurret(turret, lime));

    // oi.testJoystick.ButtonA().whenPressed(new DriveTurretWithLimelight(turret, lime));
    // oi.testJoystick.ButtonB().whenPressed(new RotateTurret(turret, 60));
    // oi.testJoystick.ButtonX().whenPressed(new RotateTurret(turret, -30));
    // oi.testJoystick.ButtonY().whenPressed(new RotateTurret(turret, -60));

    // oi.testJoystick.ButtonBack().whenPressed(new ZeroTurret(turret));

    // oi.testJoystick.ButtonLeftBumper().whenPressed(new BumpTurretLeft(turret));
    // oi.testJoystick.ButtonRightBumper().whenPressed(new BumpTurretRight(turret));

    
    // indexer.setDefaultCommand(new DriveIndexer(indexer, oi.testJoystick::getLeftStickRaw_X));
    // oi.testJoystick.ButtonRightStick().whenPressed(new ShootWithController(m_shooter, oi.testJoystick::getRightStickRaw_Y));
    // oi.testJoystick.ButtonRightStick().whenPressed(new DriveClimber(climber, oi.testJoystick::getRightStickRaw_Y));

    // oi.testJoystick.ButtonX().whenPressed(new ParallelCommandGroup(
    //         new HoodToAngle(hood, 0.0),
    //         new SetShooterSpeed(shooter, 0.0)
    // ));

    // oi.testJoystick.ButtonY().whenPressed(new HoodToAngle(hood, HoodPosition.TARMAC_LINE.position_degrees));
    // oi.testJoystick.ButtonB().whenPressed(new HoodToAngle(hood, HoodPosition.LAUNCHPAD.position_degrees));
    // oi.testJoystick.ButtonA().whenHeld(new DriveWithLimelight(drivetrain, lime, 2.0, true));

    // // oi.testJoystick.ButtonLeftDPad().whenPressed(new ExtendMonkeyBar(monkeyBar));
    // // oi.testJoystick.ButtonRightDPad().whenPressed(new RetractMonkeyBar(monkeyBar));
    // // oi.testJoystick.ButtonStart().whenPressed(new DriveClimberToZero(climber));
    // // oi.testJoystick.ButtonBack().whenPressed(new FullSendClimbingSequence(climber, monkeyBar));

    // oi.testJoystick.ButtonLeftBumper()
    //         .whileHeld(new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller))
    //         // .whileHeld(new QueueBallForShot(hopper, indexer, pooper, colorSensor, intakeRoller))
    //         .whenPressed(new IntakeLower(intakeRaiseAndLower))
    //         .whenReleased(new IntakeRaise(intakeRaiseAndLower))
    //         .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
    //         .whenReleased(new SetIntakeSpeed(intakeRoller, 0.0))
    //         .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)));

    // oi.testJoystick.ButtonRightBumper()
    //         .whileHeld(new FireBalls(shooter, indexer, hopper))
    //         //.whenPressed(new DriveHopperAndIndexer(hopper, indexer))
    //         .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
    //         .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)));

    // oi.testJoystick.ButtonLeftDPad().whenPressed(new Launchpad(hood, shooter, lime));

    // oi.testJoystick.ButtonLeftBumper().whenPressed(new PooperPoop(pooper));
    // oi.testJoystick.ButtonRightBumper().whenPressed(new PooperUnpoop(pooper));

    // oi.testJoystick.ButtonB().whenPressed(new HoodToAngle(hood, Hood.HoodPosition.TARMAC_LINE.position_degrees));
    // oi.testJoystick.ButtonB().whenPressed(new DriveTurretWithLimelight(turret, lime));
    // oi.testJoystick.ButtonA()
    // .whenPressed(new HoodToAngle(hood, Hood.HoodPosition.TWO_METER_FROM_HUB.position_degrees))
    // .whenPressed(new SetShooterSpeed(shooter, Shooter.ShooterRPM.TWO_METER_FROM_HUB));
    // oi.testJoystick.ButtonB()
    // .whenPressed(new HoodToAngle(hood, Hood.HoodPosition.THREE_METER_FROM_HUB.position_degrees))
    // .whenPressed(new SetShooterSpeed(shooter, Shooter.ShooterRPM.THREE_METER_FROM_HUB));
    // oi.testJoystick.ButtonY()
    // .whenPressed(new HoodToAngle(hood, Hood.HoodPosition.FOUR_METER_FROM_HUB.position_degrees))
    // .whenPressed(new SetShooterSpeed(shooter, Shooter.ShooterRPM.FOUR_METER_FROM_HUB));

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

  @Log
  public boolean isCompbot() {
    return Constants.IS_COMPBOT;
  }
  @Log 
  public boolean isFiring(){
    return oi.operatorJoystick.isPressedButtonRightBumper();
  }
}
