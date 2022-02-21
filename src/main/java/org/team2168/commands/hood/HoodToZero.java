// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hood;

import org.team2168.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodToZero extends CommandBase {
  /** Creates a new HoodToZero. */
  private Hood hood;
  private double descentSpeed = -5.0;

  public HoodToZero(Hood h) {
    hood = h;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.zeroDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setVelocity(descentSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      hood.setVelocity(0.0);
      hood.setEncoderPosZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hood.atZero();
  }
}
