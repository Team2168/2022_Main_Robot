// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  private static Compressor compressor = new Compressor(Constants.PneumaticsDevices.MODULE_TYPE);

  public static boolean hasMatchStarted = false;

  public Robot() {
    // set the default loop period
    super(Constants.LOOP_TIMESTEP_S);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    robotContainer = RobotContainer.getInstance();

    compressor.enableDigital();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
        
    //Only allow pushing the robot around if we aren't on a real field
    if (!DriverStation.isFMSAttached())
      robotContainer.drivetrain.setMotorsCoast();
    else
      robotContainer.drivetrain.setMotorsBrake();

    robotContainer.lime.pauseLimelight();
    Hood.getInstance().setMotorCoast();
  }

  @Override
  public void disabledPeriodic() {
    // Check periodically
    if (robotContainer.brakesEnabled())
      robotContainer.drivetrain.setMotorsCoast();
    else
      robotContainer.drivetrain.setMotorsBrake();

    // Limelight.getInstance().enableLimelight();

    if (hasMatchStarted) {
      robotContainer.drivetrain.gyroOffset = robotContainer.drivetrain.getPitch();
    }

    // TODO we probably don't want to do this
    if (Math.abs(robotContainer.drivetrain.getHeading()) > 0.5)
      robotContainer.drivetrain.zeroHeading();

    // TODO use shuffleboard here
    SmartDashboard.putString("The Actual Auto we will be running",
        robotContainer.getAutonomousCommand().getName());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    robotContainer.drivetrain.setMotorsBrakeAutos();
    Hood.getInstance().setMotorBrake();
    robotContainer.drivetrain.setMotorsBrake();
    robotContainer.lime.enableLimelight();
    autonomousCommand = robotContainer.getAutonomousCommand();

    Limelight.getInstance().enableLimelight();
    hasMatchStarted = true;

    System.out.println("scheduling auto: " + autonomousCommand.getName());
    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    Hood.getInstance().setMotorBrake();;
    robotContainer.drivetrain.setMotorsBrake();
    robotContainer.drivetrain.teleopconfigs();
    robotContainer.lime.pauseLimelight();
    hasMatchStarted = true;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
