// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.function.DoubleFunction;


import org.team2168.commands.SysIDCommand;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.hopper.DriveHopperWithVelocity;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hopper;
import org.team2168.commands.turret.*;
import org.team2168.commands.exampleSubsystem.*;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.Constants.LiftPositions;
import org.team2168.commands.*;
import org.team2168.commands.climber.*;
import org.team2168.commands.drivetrain.*;
import org.team2168.commands.hood.*;
import org.team2168.commands.indexer.*;
import org.team2168.commands.monkeybar.*;
import org.team2168.commands.pooper.*;
import org.team2168.commands.shooter.*;
import org.team2168.commands.turret.*;
import org.team2168.subsystems.*;
import org.team2168.commands.SysIDCommand;
import org.team2168.commands.IntakeRoller.IntakeSpeed;
import org.team2168.commands.climber.DriveClimberWithJoystick;
import org.team2168.commands.climber.ReturnToZero;
import org.team2168.commands.climber.SetPosition;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.hood.BumpHoodAngleDown;
import org.team2168.commands.hood.BumpHoodAngleUp;
import org.team2168.commands.hood.BumpHoodAngleZero;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.hoodAndShooter.BackOfTarmac;
import org.team2168.commands.hoodAndShooter.Terminal;
import org.team2168.commands.hoodAndShooter.WhiteLine;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.commands.pooper.PooperPoop;
import org.team2168.commands.pooper.PooperUnpoop;
import org.team2168.commands.shooter.BumpShooterSpeedDown;
import org.team2168.commands.shooter.BumpShooterSpeedUp;
import org.team2168.commands.shooter.BumpShooterSpeedZero;
import org.team2168.commands.shooter.SetShooterSpeed;
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
import org.team2168.subsystems.Hood.HoodPosition;

import edu.wpi.first.wpilibj2.command.Command;
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

  // The robot's subsystems an                           d commands are defined here...
;

  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final Pixy m_pixy = Pixy.getInstance();

  
  public final Drivetrain drivetrain = Drivetrain.getInstance();
  public final Hopper hopper = Hopper.getInstance();
  public final Pooper pooper = Pooper.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Climber climber = Climber.getInstance();
  private final Turret m_turret = Turret.getInstance();
  private final MonkeyBar monkeyBar = MonkeyBar.getInstance();
  public final IntakeRoller intakeRoller = IntakeRoller.getInstance();
  private final Indexer indexer = Indexer.getInstance();
  private final Hood hood = Hood.getInstance();
  private final IntakeRaiseAndLower intakeRAndL = IntakeRaiseAndLower.getInstance();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);


  // private ExtendExample extendExampleSubsystem= new ExtendExample(m_exampleSubsystem);
  // private RetractExample retractExampleSubsystem= new RetractExample(m_exampleSubsystem);
  // private final FindAllianceBall m_findAllianceBall = new FindAllianceBall(m_pixy);

  OI oi = OI.getInstance();

  private static RobotContainer instance = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);

    // m_pixy.setDefaultCommand(m_findAllianceBall);

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
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getDriverJoystickX, oi::getDriverJoystickY));
    


    

    //Driver Controls
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));

    //Operator Controls
    m_turret.setDefaultCommand(new DriveTurretWithJoystick(m_turret, oi.operatorJoystick::getLeftStickRaw_X));
    // climber.setDefaultCommand(new DriveClimberWithJoystick(climber, oi.operatorJoystick::getRightStickRaw_Y));
   

    oi.operatorJoystick.ButtonA().whenPressed(new ExtendMonkeyBar(monkeyBar));
    oi.operatorJoystick.ButtonA().whenReleased(new RetractMonkeyBar(monkeyBar));

    oi.operatorJoystick.ButtonX().whenHeld(new SetPosition(climber, 4.0));
    oi.operatorJoystick.ButtonY().whenPressed(new ReturnToZero(climber));

    // oi.operatorJoystick.ButtonLeftBumper().whenHeld(new HoodToAngle(hood, 0));


    //TEST JOYSTICK
    indexer.setDefaultCommand(new DriveIndexer(indexer, oi.testJoystick::getLeftStickRaw_X));
    // oi.testJoystick.ButtonRightStick().whenPressed(new ShootWithController(m_shooter, oi.testJoystick::getRightStickRaw_Y));
    oi.testJoystick.ButtonRightStick().whenPressed(new DriveClimberWithJoystick(climber, oi.testJoystick::getRightStickRaw_Y));

    oi.testJoystick.ButtonY().whenPressed(new SetPosition(climber, LiftPositions.LIFT_ABOVE_BAR_EXTENSION_INCHES));
    oi.testJoystick.ButtonB().whenPressed(new SetPosition(climber, LiftPositions.LIFT_UNLOAD_TO_MBAR_INCHES));
    oi.testJoystick.ButtonA().whenPressed(new SetPosition(climber, LiftPositions.LIFT_RETRACTION_INCHES));

    oi.testJoystick.ButtonLeftDPad().whenPressed(new ExtendMonkeyBar(monkeyBar));
    oi.testJoystick.ButtonRightDPad().whenPressed(new RetractMonkeyBar(monkeyBar));
    oi.testJoystick.ButtonStart().whenPressed(new ReturnToZero(climber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // I don't see a point of having radian conversions in our actual code, we won't
    // need them after characterization
    DoubleFunction<Double> degToRadians = (d) -> d * (Math.PI / 180.0);
    DoubleFunction<Double> ticksToRadians = (t) -> ((t / Drivetrain.TICKS_PER_REV) / Drivetrain.GEAR_RATIO) * 2.0
        * Math.PI;

    return new SysIDCommand(drivetrain, (l, r) -> drivetrain.tankDrive(l, r),
        () -> {
          return new SysIDCommand.DriveTrainSysIdData(
              ticksToRadians.apply(drivetrain.getLeftEncoderDistance()),
              ticksToRadians.apply(drivetrain.getRightEncoderDistance()),
              ticksToRadians.apply(drivetrain.getLeftEncoderRate()),
              ticksToRadians.apply(drivetrain.getRightEncoderRate()),
              degToRadians.apply(drivetrain.getHeading()),
              degToRadians.apply(drivetrain.getTurnRate()));
        }); // Drivetrain characterization

        
  }
}
