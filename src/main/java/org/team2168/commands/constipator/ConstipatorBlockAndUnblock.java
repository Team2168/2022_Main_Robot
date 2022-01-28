// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.constipator;

import org.team2168.subsystems.Constipator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConstipatorBlockAndUnblock extends CommandBase {
  /** Creates a new ConstipatorBlockAndUnblock. */
  private Constipator constipator;
  private boolean block;
  private ConstipatorBlockAndUnblock(Constipator constipator) {
    this.constipator = constipator;
    constipator = Constipator.getInstance();
    addRequirements(constipator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(block == true) {
      constipator.block();
    }
    else {
      constipator.unblock();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(block == true) {
      constipator.isBlockerBlocking();
      return true;
    }
    else {
      constipator.isBlockerNotBlocking();
      return true;
  }
}
}
