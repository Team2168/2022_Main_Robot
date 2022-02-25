// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.function.DoubleFunction;

<<<<<<< HEAD
import org.team2168.commands.DriveHopperAndIndexer;
import org.team2168.commands.SysIDCommand;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.hopper.DriveHopperWithVelocity;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hopper;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.SysIDCommand;
import org.team2168.commands.IntakeRoller.IntakeSpeed;
import org.team2168.commands.climber.DriveClimberWithJoystick;
import org.team2168.commands.climber.ReturnToZero;
import org.team2168.commands.climber.SetPosition;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.commands.pooper.PooperPoop;
import org.team2168.commands.pooper.PooperUnpoop;
import org.team2168.commands.shooter.SetSpeed;
import org.team2168.commands.shooter.ShootWithController;
import org.team2168.commands.turret.DriveTurretWithJoystick;
import org.team2168.commands.turret.RotateTurret;
import org.team2168.commands.turret.ZeroTurret;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.MonkeyBar;
import org.team2168.subsystems.Pooper;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.Turret;
=======
import org.team2168.commands.SysIDCommand;
import org.team2168.Constants.LiftPositions;
import org.team2168.commands.*;
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
>>>>>>> main

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
    //DRIVER CONTROLS
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));

<<<<<<< HEAD
    //Operator Controls
    m_turret.setDefaultCommand(new DriveTurretWithJoystick(m_turret, oi.operatorJoystick::getLeftStickRaw_X));
    climber.setDefaultCommand(new DriveClimberWithJoystick(climber, oi.operatorJoystick::getRightStickRaw_Y));
   

    oi.operatorJoystick.ButtonA().whenPressed(new ExtendMonkeyBar(monkeyBar));
    oi.operatorJoystick.ButtonA().whenReleased(new RetractMonkeyBar(monkeyBar));
    

    oi.operatorJoystick.ButtonBack().whenPressed(new RotateTurret(m_turret, 180.0));
    oi.operatorJoystick.ButtonStart().whenPressed(new RotateTurret(m_turret, 0.0));
    oi.operatorJoystick.ButtonB().whenHeld(new ZeroTurret(m_turret));
    oi.operatorJoystick.ButtonX().whenHeld(new SetPosition(climber, 12.0));
    oi.operatorJoystick.ButtonY().whenPressed(new ReturnToZero(climber));

    oi.operatorJoystick.ButtonRightBumper().whenPressed(new IntakeSpeed(intakeRoller, 0.5));
    oi.operatorJoystick.ButtonRightBumper().whenReleased(new IntakeSpeed(intakeRoller, 0.0));

    oi.operatorJoystick.ButtonLeftBumper().whenPressed(new IntakeSpeed(intakeRoller, -0.5));
    oi.operatorJoystick.ButtonLeftBumper().whenReleased(new IntakeSpeed(intakeRoller, 0.0));
    oi.operatorJoystick.ButtonRightBumper().whenHeld(new HoodToAngle(hood, 45));
    // oi.operatorJoystick.ButtonLeftBumper().whenHeld(new HoodToAngle(hood, 0));
    


    //TEST JOYSTICK
    indexer.setDefaultCommand(new DriveIndexer(indexer, oi.testJoystick::getLeftStickRaw_X));
    oi.testJoystick.ButtonRightStick().whenPressed(new ShootWithController(m_shooter, oi.testJoystick::getRightStickRaw_Y));
  

    oi.testJoystick.ButtonA().whenPressed(new SetSpeed(m_shooter, 0.0));
    oi.testJoystick.ButtonB().whenPressed(new SetSpeed(m_shooter, 216.8));
    oi.testJoystick.ButtonY().whenPressed(new SetSpeed(m_shooter, 2168.0));
    oi.testJoystick.ButtonX().whenPressed(new IntakeLower(intakeRAndL));
    oi.testJoystick.ButtonX().whenReleased(new IntakeRaise(intakeRAndL));

    oi.testJoystick.ButtonDownDPad().whenPressed(new PooperPoop(pooper));
    oi.testJoystick.ButtonDownDPad().whenReleased(new PooperUnpoop(pooper));
    
    oi.testJoystick.ButtonLeftBumper().whenPressed(new DriveHopperWithPercentOutput(hopper, () -> 0.2));
    oi.testJoystick.ButtonLeftBumper().whenReleased(new DriveHopperWithPercentOutput(hopper, () -> 0.0));
    
    oi.testJoystick.ButtonRightBumper().whenPressed(new IntakeSpeed(intakeRoller, 0.5));
    oi.testJoystick.ButtonRightBumper().whenReleased(new IntakeSpeed(intakeRoller, 0.0));

    oi.testJoystick.ButtonRightDPad().whenPressed(new DriveHopperAndIndexer(hopper, indexer, () -> 0.2));
    oi.testJoystick.ButtonRightDPad().whenReleased(new DriveHopperAndIndexer(hopper, indexer, () -> 0.0));

    oi.testJoystick.ButtonLeftDPad().whenPressed(new DriveHopperWithVelocity(hopper, () -> 20.0));
    oi.testJoystick.ButtonLeftDPad().whenReleased(new DriveHopperWithVelocity(hopper, () -> 0.0));
   
    // oi.testJoystick.ButtonLeftBumper().whenPressed(new IntakeSpeed(intakeRoller, -0.5));
    // oi.testJoystick.ButtonLeftBumper().whenReleased(new IntakeSpeed(intakeRoller, 0.0));
=======
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
            .whenPressed(new DriveHopperAndIndexer(hopper, indexer))
            .whenReleased(new DriveIndexer(indexer, () -> (0.0)))
            .whenReleased(new DriveHopperWithPercentOutput(hopper, () -> (0.0)));

    //TEST JOYSTICK
    indexer.setDefaultCommand(new DriveIndexer(indexer, oi.testJoystick::getLeftStickRaw_X));
    // oi.testJoystick.ButtonRightStick().whenPressed(new ShootWithController(m_shooter, oi.testJoystick::getRightStickRaw_Y));
    oi.testJoystick.ButtonRightStick().whenPressed(new DriveClimber(climber, oi.testJoystick::getRightStickRaw_Y));

    oi.testJoystick.ButtonY().whenPressed(new DriveClimberToPosition(climber, LiftPositions.LIFT_ABOVE_BAR_EXTENSION_INCHES));
    oi.testJoystick.ButtonB().whenPressed(new DriveClimberToPosition(climber, LiftPositions.LIFT_UNLOAD_TO_MBAR_INCHES));
    oi.testJoystick.ButtonA().whenPressed(new DriveClimberToPosition(climber, LiftPositions.LIFT_RETRACTION_INCHES));

    oi.testJoystick.ButtonLeftDPad().whenPressed(new ExtendMonkeyBar(monkeyBar));
    oi.testJoystick.ButtonRightDPad().whenPressed(new RetractMonkeyBar(monkeyBar));
    oi.testJoystick.ButtonStart().whenPressed(new DriveClimberToZero(climber));
>>>>>>> main
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
