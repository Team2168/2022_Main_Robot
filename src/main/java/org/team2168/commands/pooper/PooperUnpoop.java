// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.pooper;

import org.team2168.subsystems.Pooper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PooperUnpoop extends CommandBase {

  private Pooper pooper;
  /** Creates a new PooperUnpoop. */
  public PooperUnpoop(Pooper pooper) {
    this.pooper = pooper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pooper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pooper.backwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pooper.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
