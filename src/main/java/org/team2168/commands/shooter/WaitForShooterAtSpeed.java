// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shooter;

import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForShooterAtSpeed extends CommandBase {
  /** Creates a new WaitForShootAtSpeed. */
  private Shooter shooter;
  private double errorTolerance; //Allowable range of error
  private double loopsToSettle = 15; //The amount of loops the shooter speed needs to be within allowable error for
  private int withinThresholdLoops = 0;
  private final double DEFAULT_ERROR_TOLERANCE = 15; //in rpm
  
  public WaitForShooterAtSpeed(Shooter shooter, double errorTolerance) {
    // Doesn't require the shooter to keep the shooter being run by other commands
    this.shooter = shooter;
    this.errorTolerance = errorTolerance;
  }

  public WaitForShooterAtSpeed(Shooter shooter) {
    this.shooter = shooter;
    errorTolerance = DEFAULT_ERROR_TOLERANCE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Checks if error is within threshold*/ 
    if (shooter.isAtSpeed(errorTolerance)) {
      ++withinThresholdLoops;
    }
    else {
      withinThresholdLoops = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Shooter must stay within allowable error for 10+ loops for it to be at speed*/
    return withinThresholdLoops > loopsToSettle;
  }
}
