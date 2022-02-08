// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hood.HoodPositions;

import org.team2168.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToTestPos extends CommandBase {
  /** Creates a new MoveToTestPos. */
  Hood hood;
  
  /**
   * Moves the Hood to a test position, is currently 15 degrees
   * @param h the Hood susbsystem to use
   */
  public MoveToTestPos(Hood h) {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setPosition(30);
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
