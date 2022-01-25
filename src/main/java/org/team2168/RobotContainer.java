// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.util.function.DoubleFunction;

import org.team2168.commands.ExampleCommand;
import org.team2168.commands.SysIDCommand;
import org.team2168.commands.drivetrain.*;
import org.team2168.commands.turret.*;
import org.team2168.commands.ExampleSubsystem.*;
import org.team2168.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  public final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Turret m_turret = Turret.getInstance();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  private ExtendExample extendExampleSubsystem= new ExtendExample(m_exampleSubsystem);
  private RetractExample retractExampleSubsystem= new RetractExample(m_exampleSubsystem);

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
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));

    m_turret.setDefaultCommand(new DriveTurretWithJoystick(m_turret, oi.operatorJoystick::getLeftStickRaw_X));
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

  @Config(rowIndex = 3, columnIndex = 0, width = 1, height = 1, tabName = "ExampleSubsystem")
  private void retractExample(boolean foo) {
    retractExampleSubsystem.schedule();
  }

  @Config(rowIndex = 3, columnIndex = 1, width = 1, height = 1, tabName = "ExampleSubsystem")
  private void extendExample(boolean foo) {
    extendExampleSubsystem.schedule();
  }
}
