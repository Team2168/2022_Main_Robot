// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ResetHeading extends CommandBase {
  private Drivetrain drivetrain;
  private final double THRESHOLD = 1.0;

  /**
   * Command to Reset Gyro
   * 
   * This does not exit until the gyro has actually been reset, 
   * preventing a race condition where paths drive unexpected trajectories
   * because the path begins before the gyro has actually been reset.
   */
  public ResetHeading(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var heading = drivetrain.getHeading();
    return Math.abs(heading) > THRESHOLD;
  }
}
