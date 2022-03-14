// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CheckClimberHookAttached extends CommandBase {
  /** Creates a new CheckClimberHookAttached. */
  Climber climb;
  private static final int DEFAULT_LOOPS_TO_SETTLE = 50;
  private int loopsToSettle = 50;
  private int atPositionLoops = 0;

  /**
   * 
   * @param climber
   * @param loopsToSettle The amount of consecutive loops the climber needs to have sensors depressed for
   */
  public CheckClimberHookAttached(Climber climber, int loopsToSettle) {
    climb = climber;
    this.loopsToSettle = loopsToSettle;

    //Intentionally doesn't require the climber subsystem
  }

  public CheckClimberHookAttached(Climber climber) {
    this(climber, DEFAULT_LOOPS_TO_SETTLE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climb.isClimberHookAttached()) {
      ++atPositionLoops;
    }
    else {
      atPositionLoops = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPositionLoops > loopsToSettle;
  }
}
