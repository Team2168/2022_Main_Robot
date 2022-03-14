// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnXAngle extends CommandBase {
  private Drivetrain dt;
  private double _targetPos = 0.0;
  private double _targetAngle;

  private static final double DEFAULT_ERROR_TOLERANCE = 1.0;

  private double _errorTolerancePosition = 0.5; //0.5 inches 
  private double _errorToleranceAngle; //1.0 degree of tolerance 
  private double _loopsToSettle = 5;
  private int _withinThresholdLoops = 0;

  public TurnXAngle(Drivetrain dt, double setPoint) {
    this(dt, setPoint, DEFAULT_ERROR_TOLERANCE);
  }

  public TurnXAngle(Drivetrain dt, double setPoint, double errorToleranceAngle) {
    this.dt = dt;
    addRequirements(dt);

    _errorToleranceAngle = errorToleranceAngle;
    _targetAngle = setPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _withinThresholdLoops = 0;
    dt.zeroHeading();
    dt.resetEncoders();
    dt.switchGains(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.setSetPointPosition(_targetPos, _targetAngle);
    /* Check if closed loop error is within the threshld */
    if ((Math.abs(dt.getErrorPosition()) < _errorTolerancePosition) && (Math.abs(dt.getErrorHeading()) < _errorToleranceAngle)) {
      ++_withinThresholdLoops;
    } else {
      _withinThresholdLoops = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _withinThresholdLoops > _loopsToSettle;
  }
}
