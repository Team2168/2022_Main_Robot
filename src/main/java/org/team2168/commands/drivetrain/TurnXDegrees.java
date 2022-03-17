// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnXDegrees extends CommandBase {
  /** Creates a new DriveWithLimelight. */
  private Drivetrain dt;
  private PIDController pid;

  private static double DEFAULT_MAXANGLE = 1.0;
  private double errorToleranceAngle; // in degrees
  private double targetHeading;
  private int withinThresholdLoops = 0;
  private int acceptableLoops = 10;

  private static final double MINIMUM_COMMAND = 0.05;
  private static final double MAX_INTEGRAL = 0.1;
  
  //limelight gains
  private double P = 0.0041;
  private double I = 0.001;
  private double D = 0.0;

  private double turnSpeed;

  public TurnXDegrees(Drivetrain drivetrain, double targetHeading) {
    this(drivetrain, targetHeading, DEFAULT_MAXANGLE);
  }

  public TurnXDegrees(Drivetrain drivetrain, double targetHeading, double acceptableAngle) {
    dt = drivetrain;
    errorToleranceAngle = acceptableAngle;
    this.targetHeading = targetHeading;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(P, I, D);

    pid.setTolerance(errorToleranceAngle);
    pid.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);

    dt.zeroHeading();
    dt.resetEncoders();
    dt.teleopconfigs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = dt.getHeading();
    double error = currentHeading - targetHeading;
  
    if (Math.abs(error) < errorToleranceAngle) {
      ++withinThresholdLoops;
    } else {
      withinThresholdLoops = 0;
    }

    if (error < -errorToleranceAngle) {
      turnSpeed = -(pid.calculate(error) + MINIMUM_COMMAND);
    } else if (error > errorToleranceAngle) {
      turnSpeed = -(pid.calculate(error) - MINIMUM_COMMAND);
    } else {
      turnSpeed = 0.0;
    }

    // System.out.println("   tgt:" + targetHeading + ", curr:" + currentHeading + ", err:" + error + ", spd:" + turnSpeed);
    dt.arcadeDrive(0.0, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (withinThresholdLoops >= acceptableLoops); // command does not need to finish if bound to a button
  }
}
