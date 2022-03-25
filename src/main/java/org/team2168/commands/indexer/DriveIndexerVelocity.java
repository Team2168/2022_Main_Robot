// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.indexer;

import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIndexerVelocity extends CommandBase {
  /** Creates a new DriveIndexerVelocity. */
  private Indexer indexer;
  private double RPM_speed;

  public DriveIndexerVelocity(Indexer indexer, double RPM_speed) {
    this.indexer = indexer;
    this.RPM_speed = RPM_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.driveVelocity(RPM_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.drive(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
