// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hopper;

import org.team2168.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroHopper extends CommandBase {
  private Hopper hopper;

  /** Creates a new ZeroHopper. */
  public ZeroHopper(Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;

    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.zeroEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.driveHopper(0);
    if (!interrupted) {
      hopper.zeroEncoder();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
