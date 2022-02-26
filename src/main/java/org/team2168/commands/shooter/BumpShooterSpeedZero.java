// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.shooter;

import org.team2168.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpShooterSpeedZero extends CommandBase {
  /** Creates a new BumpShooterSpeedZero. */
  private Shooter shooter;

  /**
   * Sets the bump amount to zero
   * @param s the shooter instance
   */
  public BumpShooterSpeedZero(Shooter s) {
    shooter = s;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.zeroSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
