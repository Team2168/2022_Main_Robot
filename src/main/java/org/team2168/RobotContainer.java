// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import java.io.IOException;
import java.util.function.DoubleFunction;

import org.team2168.commands.SysIDCommand;
import org.team2168.commands.auto.DebugPath;
import org.team2168.commands.climber.DriveClimberWithJoystick;
import org.team2168.commands.climber.ReturnToZero;
import org.team2168.commands.climber.SetPosition;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.exampleSubsystem.ExtendExample;
import org.team2168.commands.exampleSubsystem.RetractExample;
import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.commands.pixy.FindAllianceBall;
import org.team2168.commands.turret.DriveTurretWithJoystick;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.MonkeyBar;
import org.team2168.subsystems.Pixy;
import org.team2168.subsystems.Turret;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  private static RobotContainer instance = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
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

    //Operator Controls
    // m_turret.setDefaultCommand(new DriveTurretWithJoystick(m_turret, oi.operatorJoystick::getLeftStickRaw_X));
    // climber.setDefaultCommand(new DriveClimberWithJoystick(climber, oi.operatorJoystick::getLeftStickRaw_Y));

    // oi.operatorJoystick.ButtonA().whenPressed(new ExtendMonkeyBar(monkeyBar));
    // oi.operatorJoystick.ButtonA().whenReleased(new RetractMonkeyBar(monkeyBar));
    // oi.operatorJoystick.ButtonX().whenHeld(new SetPosition(climber, 12.0));
    // oi.operatorJoystick.ButtonY().whenPressed(new ReturnToZero(climber));
  }

  private void configureAutonomousRoutines() {
    var drive1Meter = PathUtil.getPathCommand("Drive1Meter", drivetrain, InitialPathState.DISCARDHEADING);
    var lShape = new SequentialCommandGroup();
    var squiggles = PathUtil.getPathCommand("Squiggles", drivetrain, InitialPathState.DISCARDHEADING);
    var drive3Meters= PathUtil.getPathCommand("Drive3Meters", drivetrain, InitialPathState.DISCARDHEADING);
    var drive5MSquiggles=PathUtil.getPathCommand("5MSquiggles", drivetrain, InitialPathState.DISCARDHEADING);


    autoChooser.setDefaultOption("Do nothing", new InstantCommand());
    autoChooser.addOption("Drive 1 Meter", drive1Meter);
    autoChooser.addOption("LShape", lShape);
    autoChooser.addOption("Squiggles", squiggles);
    autoChooser.addOption("Debug auto", new DebugPath(drivetrain, "Drive3Meters"));
    autoChooser.addOption("Drive 3 Meters", drive3Meters);
    //Test Path that goes 2 meters to the right (y-axis) and 4.5 meters forward (x-axis)in a "U" shape"
    autoChooser.addOption("Drive 5 Squiggle", drive5MSquiggles);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
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
}