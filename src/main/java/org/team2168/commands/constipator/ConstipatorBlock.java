// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.constipator;

import org.team2168.subsystems.Constipator;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConstipatorBlock extends CommandBase {
  /** Creates a new ConstipatorBlock. */
  private Constipator _constipator;
  public ConstipatorBlock() {
    // Use addRequirements() here to declare subsystem dependencies.
    _constipator = Constipator.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _constipator.block();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _constipator.isBlockerBlocking();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
