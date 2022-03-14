// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveXDistance extends CommandBase {

  private Drivetrain dt;
  /**target position */
  private double _targetPos;
  private double _targetAngle = 0.0;

  private static final double DEFAULT_ERROR_TOLERANCE = 0.1; //inches
  private static final double DEFAULT_MAX_VEL = 10.0*12.0;
  private static final double DEFAULT_LOOPS_TO_SETTLE = 5;

  private double _errorTolerancePosition; //0.5 inches
  private double _errorToleranceAngle = 3.0; //1.0 degree of tolerance 
  private double _maxVel;
  private double _loopsToSettle;
  private int _withinThresholdLoops = 0;

  /** Creates a new DriveXDisatance. */
  /**
   * 
   * @param dt subsystem instance
   * @param setPoint the target position in inches
   */
  public DriveXDistance(Drivetrain dt, double setPoint) {
    this(dt, setPoint, DEFAULT_ERROR_TOLERANCE, DEFAULT_MAX_VEL, DEFAULT_LOOPS_TO_SETTLE);
  }

  /**
   * 
   * @param dt subsystem instance
   * @param setPoint the target position in inches
   * @param errorTolerancePosition accepatable error in inches
   */
  public DriveXDistance(Drivetrain dt, double setPoint, double errorTolerancePosition) {
    this(dt, setPoint, errorTolerancePosition, DEFAULT_MAX_VEL, DEFAULT_LOOPS_TO_SETTLE);
  }

  /**
   * 
   * @param dt subsystem instance
   * @param setPoint the target position in inches
   * @param errorTolerancePosition accepatable error in inches
   * @param maxVelocity max speed in inches/sec
   */
  public DriveXDistance(Drivetrain dt, double setPoint, double errorTolerancePosition, double maxVelocity) {
    this(dt, setPoint, errorTolerancePosition, maxVelocity, DEFAULT_LOOPS_TO_SETTLE);
  }

  public DriveXDistance(Drivetrain dt, double setPoint, double errorTolerancePosition, double maxVelocity, double loopsToSettle) {
    this.dt = dt;
    addRequirements(dt);

    _errorTolerancePosition = errorTolerancePosition;
    _targetPos = setPoint;
    _maxVel = maxVelocity;
    _loopsToSettle = loopsToSettle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.zeroHeading();
    dt.resetEncoders();
    dt.switchGains(true);
    dt.setCruiseVelocity(_maxVel);
    _withinThresholdLoops = 0;
    dt.setSetPointPosition(_targetPos, _targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Check if closed loop error is within the threshld */

    System.out.println("  posErr: " + dt.getErrorPosition() + "  ang: " + dt.getErrorHeading());

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
