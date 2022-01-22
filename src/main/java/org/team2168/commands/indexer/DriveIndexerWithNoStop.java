// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.indexer;

import org.team2168.Robot;
import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIndexerWithNoStop extends CommandBase {
  /** Creates a new DriveIndexerWithNoStop. */
  private double _speed;
  private Indexer _indexer;
  public DriveIndexerWithNoStop() {
    // Use addRequirements() here to declare subsystem dependencies.
    _indexer = Indexer.getInstance();
  
    addRequirements(_indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_speed != 0.0) {
      Robot.setCompressorOn(false);
    }
    else {
      Robot.setCompressorOn(true);
    }
    _indexer.drive(_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.setCompressorOn(true);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
