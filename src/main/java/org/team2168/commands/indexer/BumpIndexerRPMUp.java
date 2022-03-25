// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.indexer;

import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpIndexerRPMUp extends CommandBase {
  /** Creates a new BumpIndexerRPMUp. */
  private Indexer indexer;

  /**
   * Bumps the indexer RPM up by 50 RPM
   * @param indexer
   */
  public BumpIndexerRPMUp(Indexer indexer) {
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = indexer.getSetpoint() + 50;
    indexer.driveVelocity(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
