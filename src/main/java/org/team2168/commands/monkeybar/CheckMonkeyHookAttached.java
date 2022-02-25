// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.monkeybar;

import org.team2168.subsystems.MonkeyBar;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CheckMonkeyHookAttached extends CommandBase {
  /** Creates a new CheckMonkeyHookAttached. */
  MonkeyBar monkey;
  public CheckMonkeyHookAttached(MonkeyBar monkeyBar) {
    // Use addRequirements() here to declare subsystem dependencies.
    monkey = monkeyBar;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return monkey.isHookEngaged();
  }
}
