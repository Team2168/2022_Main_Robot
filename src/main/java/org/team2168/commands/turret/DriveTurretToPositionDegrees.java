// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Turret;
import org.team2168.utils.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTurretToPositionDegrees extends CommandBase {
  /** Creates a new RotateTurret. */
  private Turret turret;
  private double targetPositionDegrees;
  private double acceptableErrorDegrees = 0.1;

  private double error;

  /**
   * Rotates the turret to a desired absolute heading
   * @param t the turret subsystem to be used
   * @param targetPosition the desired absolute position of the turret (degrees)
   */

  public DriveTurretToPositionDegrees(Turret t, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = t;
    targetPositionDegrees = targetPosition;

    addRequirements(t);
  }

  /**
   * Rotates the turret to a desired absolute heading
   * @param t the turret subsystem to be used
   * @param targetPosition the desired absolute position of the turret (degrees)
   * @param acceptableError how close to the target posotion do we need to be to finish the command (degrees)
   */
  public DriveTurretToPositionDegrees(Turret t, double targetPosition, double acceptableError) {
    // Use addRequirements() here to declare subsystem dependencies.
    acceptableErrorDegrees = acceptableError;
    addRequirements(t);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    error = turret.getPositionDegrees() - targetPositionDegrees;
    //position = turret.getEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setRotationDegrees(targetPositionDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    //Checks if the current position of the turret is where it should be or is close to where it should be 
    double currentError = turret.getPositionDegrees() - targetPositionDegrees;
    error = Util.runningAverage(currentError, error, 0.85);

    return Math.abs(error) < acceptableErrorDegrees;
  }
}
