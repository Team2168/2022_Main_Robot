// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.annotations.Log;

public class DriveTurretWithLimelight extends CommandBase {
  /** Creates a new DriveTurretWithLimelight. */

  private Turret turret;
  private Limelight limelight;
  private PIDController pid;

  private double errorToleranceAngle = 1; // in degrees
  private double limeXPos;
  private int withinThresholdLoops = 0;
  private int acceptableLoops = 10;

  private double currentPosition;
  private double targetPos = currentPosition + limeXPos;

  private static final double MINIMUM_COMMAND = 0.25;  // TODO normalize for battery voltage
  private static final double MAX_INTEGRAL = 1.0;
  
  //limelight gains
  private double P;
  private double I;
  private double P_FAR = 0.02;
  private double P_NEAR = 0.01;
  private double I_FAR = 0.002;
  private double I_NEAR = 0.001;
  private double D = 0.0;

  boolean isNear = true;

  @Log(name = "Turn Speed")
  private double driveLimeTurnSpeed;

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

  public DriveTurretWithLimelight(Turret turret, Limelight limelight, double acceptableAngle, boolean isNear) {
    this.turret = turret;
    this.limelight= limelight;
    errorToleranceAngle = acceptableAngle;

    if(isNear) {
      P = P_NEAR;
      I = I_NEAR;
    }
    else {
      P = P_FAR;
      I = I_FAR;
    }

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPosition = turret.getPositionDegrees();

    pid = new PIDController(P, I, D);
    pid.setTolerance(errorToleranceAngle);
    pid.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);

    limelight.enableLimelight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //How far away the target is in degrees
    limeXPos = limelight.getPositionX();

    if (-360 < targetPos && targetPos < 360) {
      if (limeXPos < -errorToleranceAngle)
      driveLimeTurnSpeed = -(pid.calculate(limeXPos) + MINIMUM_COMMAND);
    else if (limeXPos > errorToleranceAngle)
      driveLimeTurnSpeed = -(pid.calculate(limeXPos) - MINIMUM_COMMAND);
    else  
      driveLimeTurnSpeed = 0.0;
    }
    
    //TODO: change this to include errorToleranceAngle
    else {
      driveLimeTurnSpeed = - currentPosition + turret.amountFromZeroToRotate(targetPos);
    }

    
    turret.setVelocity(driveLimeTurnSpeed);

    // should be last thing(?)
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
