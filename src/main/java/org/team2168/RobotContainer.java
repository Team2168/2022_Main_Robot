// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import io.github.oblarg.oblog.Logger;
import org.team2168.commands.climber.DriveClimberWithJoystick;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.turret.DriveTurretWithJoystick;
import org.team2168.subsystems.*;

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
  private final Turret turret = Turret.getInstance();
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
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getDriverJoystickX, oi::getDriverJoystickY));

    //Driver Controls
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));

    //Operator Controls
    turret.setDefaultCommand(new DriveTurretWithJoystick(turret, oi.operatorJoystick::getLeftStickRaw_X));
    climber.setDefaultCommand(new DriveClimberWithJoystick(climber, oi.operatorJoystick::getRightStickRaw_Y));
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
