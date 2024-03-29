// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.intakeraiseandlower;

import org.team2168.subsystems.IntakeRaiseAndLower;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeRaise extends CommandBase {
  private IntakeRaiseAndLower intake;

  /** Creates a new IntakeRaise. */
  public IntakeRaise(IntakeRaiseAndLower intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.raise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isIntakeRaised();
  }
}
