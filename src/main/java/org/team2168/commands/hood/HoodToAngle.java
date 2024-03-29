// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hood;

import org.team2168.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodToAngle extends CommandBase {
  /** Creates a new HoodToAngle. */
  private Hood hood;
  private double angle;

  /**
   * Sets the hood to an angle
   * @param h the hood instance
   * @param a the angle to go to
   */
  public HoodToAngle(Hood h, double a) {
    hood = h;
    angle = a;

    addRequirements(h);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setPosition(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
