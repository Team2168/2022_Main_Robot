// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.function.DoubleFunction;


import org.team2168.commands.SysIDCommand;
import org.team2168.commands.climber.*;
import org.team2168.commands.drivetrain.*;
import org.team2168.commands.hood.*;
import org.team2168.commands.hoodAndShooter.*;
import org.team2168.commands.indexer.*;
import org.team2168.commands.intakeroller.*;
import org.team2168.commands.intakeraiseandlower.*;
import org.team2168.commands.monkeybar.*;
import org.team2168.commands.pooper.*;
import org.team2168.commands.shooter.*;
import org.team2168.commands.turret.*;
import org.team2168.subsystems.*;
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

    if (Constants.IS_PRACTICEBOT)
      System.out.println("FOUND PRACTICE BOT JUMPER -- CONFIGURING WITH PBOT GAINS");
    else
      System.out.println("DID NOT FIND PRACTICE BOT JUMPER -- USING COMPETITION BOT GAINS");
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
    climber.setDefaultCommand(new DriveClimberWithJoystick(climber, oi.operatorJoystick::getRightStickRaw_Y));
   

    oi.operatorJoystick.ButtonA().whenPressed(new ExtendMonkeyBar(monkeyBar));
    oi.operatorJoystick.ButtonA().whenReleased(new RetractMonkeyBar(monkeyBar));
    oi.operatorJoystick.ButtonBack().whenPressed(new RotateTurret(m_turret, 180.0));
    oi.operatorJoystick.ButtonStart().whenPressed(new RotateTurret(m_turret, 0.0));
    oi.operatorJoystick.ButtonB().whenHeld(new ZeroTurret(m_turret));
    oi.operatorJoystick.ButtonX().whenHeld(new SetPosition(climber, 12.0));
    oi.operatorJoystick.ButtonY().whenPressed(new ReturnToZero(climber));

    oi.operatorJoystick.ButtonRightBumper().whenHeld(new HoodToAngle(hood, 45));
    // oi.operatorJoystick.ButtonLeftBumper().whenHeld(new HoodToAngle(hood, 0));


    oi.operatorJoystick.ButtonA().whenHeld(new HoodToAngle(hood, HoodPosition.BACK_OF_TARMAC.position_degrees));
    oi.operatorJoystick.ButtonB().whenHeld(new HoodToAngle(hood, HoodPosition.WHITE_LINE.position_degrees));
    oi.operatorJoystick.ButtonY().whenHeld(new HoodToAngle(hood, HoodPosition.TERMINAL.position_degrees));

    oi.operatorJoystick.ButtonLeftBumper().whenPressed(new BumpHoodAngleDown(hood));
    oi.operatorJoystick.ButtonRightBumper().whenPressed(new BumpHoodAngleUp(hood));
    oi.operatorJoystick.ButtonUpDPad().whenPressed(new BumpHoodAngleZero(hood));

    //TEST JOYSTICK
    indexer.setDefaultCommand(new DriveIndexer(indexer, oi.testJoystick::getLeftStickRaw_X));
    oi.testJoystick.ButtonRightStick().whenPressed(new ShootWithController(m_shooter, oi.testJoystick::getRightStickRaw_Y));
  

    oi.testJoystick.ButtonX().whenPressed(new SetShooterSpeed(m_shooter, 0.0));

    oi.testJoystick.ButtonA().whenPressed(new BackOfTarmac(hood, m_shooter));
    oi.testJoystick.ButtonB().whenPressed(new WhiteLine(hood, m_shooter));
    oi.testJoystick.ButtonY().whenPressed(new Terminal(hood, m_shooter));

    oi.testJoystick.ButtonRightBumper().whenPressed(new BumpShooterSpeedUp(m_shooter));
    oi.testJoystick.ButtonLeftBumper().whenPressed(new BumpShooterSpeedDown(m_shooter));
    oi.testJoystick.ButtonUpDPad().whenPressed(new BumpShooterSpeedZero(m_shooter));

    //oi.testJoystick.ButtonA().whenPressed(new SetSpeed(m_shooter, 0.0));
    oi.testJoystick.ButtonX().whenPressed(new IntakeLower(intakeRAndL));
    oi.testJoystick.ButtonX().whenReleased(new IntakeRaise(intakeRAndL));

    oi.testJoystick.ButtonLeftBumper().whenPressed(new PooperPoop(pooper));
    oi.testJoystick.ButtonLeftBumper().whenReleased(new PooperUnpoop(pooper));

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
