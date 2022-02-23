// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hood;

import org.team2168.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpHoodAngleZero extends CommandBase {
  /** Creates a new BumpHoodAngleZero. */
  public Hood hood;

  /**
   * Sets the bump amount of the hood to zero
   * @param h the hood instance
   */
  public BumpHoodAngleZero(Hood h) {
    hood = h;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.zeroDegrees();
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
