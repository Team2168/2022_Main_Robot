// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.pooper;

import org.team2168.subsystems.Pooper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PooperPoop extends CommandBase {
  /** Creates a new PooperPoop. */

  private Pooper pooper;
 



  public PooperPoop(Pooper pooper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pooper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pooper.forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pooper.backwards();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
