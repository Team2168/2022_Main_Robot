// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.annotations.Log;

public class DriveTurretWithLimelight extends CommandBase {
  /** Creates a new DriveTurretWithLimelight. */

  private Turret turret;
  private Limelight limelight;

  private double errorToleranceAngle = 1; // in degrees
  private double limeXPos;
  private int withinThresholdLoops = 0;
  private int acceptableLoops = 10;

  private double currentPos;
  private double targetPos = currentPos + limeXPos;

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

    limelight.enableLimelight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //How far away the target is horizontally in degrees
    limeXPos = limelight.getPositionX();
    currentPos = turret.getPositionDegrees();


    // if the target is within the soft limits
    if (-360 < targetPos && targetPos < 360) {
      if (limeXPos < -errorToleranceAngle || limeXPos > errorToleranceAngle)
        driveLimeTurn = limeXPos;
      else  
        driveLimeTurn = 0.0;
    }
    
    else {
      driveLimeTurn = - currentPos + turret.amountFromZeroToRotate(targetPos);
    }

    turret.setRotationDegrees(driveLimeTurn);

    if (Math.abs(limeXPos) < errorToleranceAngle) 
      ++ withinThresholdLoops;
    else
      withinThresholdLoops = 0;
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
    return (withinThresholdLoops >= acceptableLoops);
  }
}
