// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.function.DoubleFunction;

import org.team2168.commands.SysIDCommand;
import org.team2168.commands.IntakeRoller.IntakeSpeed;
import org.team2168.commands.climber.DriveClimberWithJoystick;
import org.team2168.commands.climber.ReturnToZero;
import org.team2168.commands.climber.SetPosition;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.commands.turret.DriveTurretWithJoystick;
import org.team2168.commands.turret.RotateTurret;
import org.team2168.commands.turret.ZeroTurret;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.MonkeyBar;
import org.team2168.subsystems.Turret;

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
  private final Climber climber = Climber.getInstance();
  private final Turret m_turret = Turret.getInstance();
  private final MonkeyBar monkeyBar = MonkeyBar.getInstance();
  public final IntakeRoller intakeRoller = IntakeRoller.getInstance();

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

    oi.operatorJoystick.ButtonRightBumper().whenPressed(new IntakeSpeed(intakeRoller, 0.5));
    oi.operatorJoystick.ButtonRightBumper().whenReleased(new IntakeSpeed(intakeRoller, 0.0));

    oi.operatorJoystick.ButtonLeftBumper().whenPressed(new IntakeSpeed(intakeRoller, -0.5));
    oi.operatorJoystick.ButtonLeftBumper().whenReleased(new IntakeSpeed(intakeRoller, 0.0));
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
