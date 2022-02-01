// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.intakeraiseandlower;

import org.team2168.subsystems.IntakeRaiseAndLower;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerAndRaise extends CommandBase {
  /** Creates a new LowerAndRaise. */
  private IntakeRaiseAndLower intake;
  private boolean lower;
  public LowerAndRaise(IntakeRaiseAndLower intake) {
    this.intake = intake;
    intake = IntakeRaiseAndLower.getInstance();
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lower == true) {
      intake.lower();
    }
    else {
      intake.raise();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(lower == true) {
      intake.isIntakeLowered();
      return true;
    }
    else {
      intake.isIntakeRaised();
      return true;
    }
    
  }
}
