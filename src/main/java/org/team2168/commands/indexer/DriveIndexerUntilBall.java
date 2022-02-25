// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.indexer;

import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveIndexerUntilBall extends CommandBase {
  private double indexerSpeed;
  private Indexer indexer;

  public DriveIndexerUntilBall(Indexer indexer, DoubleSupplier speed) {
    this.indexer = indexer;
    this.indexerSpeed = speed.getAsDouble();
    addRequirements(indexer);
  }

  public DriveIndexerUntilBall(Indexer indexer, double speed) {
    this.indexer = indexer;
    indexerSpeed = speed;
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
    return indexer.isBallPresent();
  }
}
