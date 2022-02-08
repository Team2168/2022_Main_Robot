// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hood.HoodPositions;

import org.team2168.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToTestPos1 extends CommandBase {
  /** Creates a new MoveToTestPos1. */
  Hood hood;
  public MoveToTestPos1(Hood h) {
    hood = h;
    addRequirements(h);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setPosition(45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
