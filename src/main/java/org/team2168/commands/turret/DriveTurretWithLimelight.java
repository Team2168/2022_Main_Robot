// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;
import org.team2168.utils.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.annotations.Log;

public class DriveTurretWithLimelight extends CommandBase {
  /** Creates a new DriveTurretWithLimelight. */

  private Turret turret;
  private Limelight limelight;

  private double errorToleranceAngle = 1.0; // in degrees
  private double limeXPos;
  private double avg_limeXPos;

  private double currentPos;
  private double targetPos;

  private double forwardSoftLimit;
  private double reverseSoftLimit;

  private boolean unwinding = false;
  private double unwind_target;
  private static final double UNWIND_TOLERANCE_DEGREES = 20.0;

  private static final double LIME_KP = 0.65; 

  @Log(name = "Turn Speed")
  private double driveLimeTurn;

  /**
   * Drives the turret with the limelight for targeting 
   * @param turret the turret instance
   * @param limelight the limelight instance
   */
  public DriveTurretWithLimelight(Turret turret, Limelight limelight) {
    this.turret = turret;
    this.limelight = limelight;

    addRequirements(turret);
  }

  /**
   * Drives the turret with the limelight for targeting 
   * @param turret the turret instance
   * @param limelight the limelight instance
   * @param acceptableAngle the distance in degrees the turret is allowed to be off by
   */
  public DriveTurretWithLimelight(Turret turret, Limelight limelight, double acceptableAngle) {
    this.turret = turret;
    this.limelight= limelight;
    errorToleranceAngle = acceptableAngle;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPos = turret.getPositionDegrees();
    forwardSoftLimit = turret.getForwardSoftLimit();
    reverseSoftLimit = turret.getReverseSoftLimit();
    avg_limeXPos = 0.0;
    limelight.enableLimelight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //How far away the target is horizontally in degrees
    limeXPos = limelight.getPositionX();
    avg_limeXPos = Util.runningAverage(limeXPos, avg_limeXPos, 0.15);
    currentPos = turret.getPositionDegrees();
    targetPos = currentPos + (avg_limeXPos * LIME_KP);

    if(unwinding) {
      //We are in the middle of rotating a full 360.0 we are going to lose view of the target
      //  wait till we get where we were going
      if(Math.abs(unwind_target - currentPos) <= UNWIND_TOLERANCE_DEGREES) {
        unwinding = false;
      }
    } else if ((targetPos > reverseSoftLimit) && (targetPos < forwardSoftLimit)) {
      // if the target is within the soft limits
      if (Math.abs(avg_limeXPos) >= errorToleranceAngle) {
        driveLimeTurn = targetPos;
      } else {  
        driveLimeTurn = currentPos;
      }
    } else {
      driveLimeTurn = turret.amountFromZeroToRotate(targetPos);
      unwind_target = driveLimeTurn;
      unwinding = true;
    }

    turret.setRotationDegrees(driveLimeTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0.0);
    limelight.pauseLimelight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
