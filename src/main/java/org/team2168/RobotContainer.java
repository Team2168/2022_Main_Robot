// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.function.DoubleFunction;

import org.team2168.Constants.LiftPositions;
import org.team2168.commands.*;
import org.team2168.commands.LEDs.SetBlueLED;
import org.team2168.commands.LEDs.SetGreenLED;
import org.team2168.commands.LEDs.SetRedLED;
import org.team2168.commands.LEDs.ShowShooterAtSpeed;
import org.team2168.commands.climber.*;
import org.team2168.commands.drivetrain.*;
import org.team2168.commands.hood.*;
import org.team2168.commands.indexer.*;
import org.team2168.commands.intakeroller.*;
import org.team2168.commands.intakeraiseandlower.*;
import org.team2168.commands.monkeybar.*;
import org.team2168.commands.pooper.*;
import org.team2168.commands.shooter.*;
import org.team2168.commands.shootingpositions.*;
import org.team2168.commands.hopper.*;
import org.team2168.commands.turret.*;
import org.team2168.subsystems.*;
import org.team2168.subsystems.Hood.HoodPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  // The robot's subsystems and commands are defined here...

  public final Drivetrain drivetrain = Drivetrain.getInstance();
  public final Hopper hopper = Hopper.getInstance();
  public final Pooper pooper = Pooper.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();
  // private final Turret turret = Turret.getInstance(); // motor not powered for time being
  private final MonkeyBar monkeyBar = MonkeyBar.getInstance();
  public final IntakeRoller intakeRoller = IntakeRoller.getInstance();
  private final Indexer indexer = Indexer.getInstance();
  private final Hood hood = Hood.getInstance();
  private final IntakeRaiseAndLower intakeRaiseAndLower= IntakeRaiseAndLower.getInstance();
  private final LEDs leds = LEDs.getInstance();


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
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    leds.setDefaultCommand(new ShowShooterAtSpeed(leds, shooter));

    //DRIVER CONTROLS
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));

    //// Green button
    // oi.driverJoystick.ButtonLeftStick()
    //        .whenPressed(new DriveWithLimelight(drivetrain))
    //        .whenReleased(new ArcadeDrive(drivetrain, () -> 0.0, () -> 0.0));

    //// Black button
    // oi.driverJoystick.ButtonRightBumper().whenPressed(new AutoClimbFullSend());

    //// Red button
    oi.driverJoystick.ButtonA().whenPressed(new StowEverything(hood));


    //OPERATOR CONTROLS
    //// main button cluster
    oi.operatorJoystick.ButtonA().whenPressed(new FenderLow(hood, shooter));
    oi.operatorJoystick.ButtonB().whenPressed(new TarmacLine(hood, shooter));
    oi.operatorJoystick.ButtonX().whenPressed(new Launchpad(hood, shooter));
    oi.operatorJoystick.ButtonY().whenPressed(new FenderHigh(hood, shooter));
    oi.operatorJoystick.ButtonRightTrigger().whenPressed(new WallShot(hood, shooter));

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

    //// Trigger cluster
    oi.operatorJoystick.ButtonLeftBumper()
            .whenPressed(new LowerAndRunIntake(intakeRaiseAndLower, intakeRoller, hopper, indexer))
            .whenReleased(new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller, hopper, indexer));
    oi.operatorJoystick.ButtonRightBumper()
            .whileHeld(new FireBalls(shooter, indexer, hopper))
            //.whenPressed(new DriveHopperAndIndexer(hopper, indexer))
            .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
            .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)));

    //TEST JOYSTICK
    indexer.setDefaultCommand(new DriveIndexer(indexer, oi.testJoystick::getLeftStickRaw_X));
    // oi.testJoystick.ButtonRightStick().whenPressed(new ShootWithController(m_shooter, oi.testJoystick::getRightStickRaw_Y));
    oi.testJoystick.ButtonRightStick().whenPressed(new DriveClimber(climber, oi.testJoystick::getRightStickRaw_Y));

    oi.testJoystick.ButtonY().whenPressed(new DriveClimberToPosition(climber, LiftPositions.LIFT_ABOVE_BAR_EXTENSION_INCHES));
    oi.testJoystick.ButtonB().whenPressed(new DriveClimberToPosition(climber, LiftPositions.LIFT_UNLOAD_TO_MBAR_INCHES));
    oi.testJoystick.ButtonA().whenPressed(new DriveClimberToPosition(climber, LiftPositions.LIFT_RETRACTION_INCHES));

    // oi.testJoystick.ButtonA()
    //         .whenPressed(new SetRedLED(leds, true))
    //         .whenReleased(new SetRedLED(leds, false));
          
    // oi.testJoystick.ButtonB()
    //         .whenPressed(new SetGreenLED(leds, true))
    //         .whenReleased(new SetGreenLED(leds, false));

    // oi.testJoystick.ButtonX()
    //         .whenPressed(new SetBlueLED(leds, true))
    //         .whenReleased(new SetBlueLED(leds, false));

    oi.testJoystick.ButtonLeftDPad().whenPressed(new ExtendMonkeyBar(monkeyBar));
    oi.testJoystick.ButtonRightDPad().whenPressed(new RetractMonkeyBar(monkeyBar));
    oi.testJoystick.ButtonStart().whenPressed(new DriveClimberToZero(climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
