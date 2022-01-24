// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.VerticalClimber;

import org.team2168.subsystems.VerticalClimber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPosition extends CommandBase {
  /** Creates a new SetPosition. */
  VerticalClimber verticalClimber = VerticalClimber.getInstance();
  double inches;
  public SetPosition(VerticalClimber verticalClimber, double inch) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(verticalClimber);
    inches = inch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    verticalClimber.setPosition(inches);
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
