// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.indexer;

import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIndexerUntilNoBall extends CommandBase {
  /** Creates a new DriveUntilNoBall. */
  private Indexer indexer;
  private double indexerSpeed;

  public DriveIndexerUntilNoBall(Indexer indexer, double speed) {
    this.indexer = indexer;
    indexerSpeed = speed;

    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.drive(indexerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.drive(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Waits until there is no ball
    return !indexer.isBallPresent();
  }
}
