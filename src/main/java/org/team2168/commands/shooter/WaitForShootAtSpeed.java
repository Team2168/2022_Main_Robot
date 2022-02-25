// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shooter;

import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForShootAtSpeed extends CommandBase {
  /** Creates a new WaitForShootAtSpeed. */
  private Shooter shooter;
  private double errorTolerance;
  private double loopsToSettle = 10;
  private int withinThresholdLoops = 0;
  private final double DEFAULT_ERROR_TOLERANCE = 100;
  
  public WaitForShootAtSpeed(Shooter shooter, double errorTolerance) {
    this.shooter = shooter;
    this.errorTolerance = errorTolerance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public WaitForShootAtSpeed(Shooter shooter) {
    this.shooter = shooter;
    errorTolerance = DEFAULT_ERROR_TOLERANCE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(shooter.gerError()) < errorTolerance) {
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
    return withinThresholdLoops > loopsToSettle;
  }
}
