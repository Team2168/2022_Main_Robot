// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.indexer;

import org.team2168.OI;
import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIndexerWithJoystick extends CommandBase {
  /** Creates a new DriveIndexerWithJoystick. */
  private Indexer _indexer;
  private OI _oi;
  public DriveIndexerWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    _indexer = Indexer.getInstance();

    addRequirements(_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _oi = OI.getInstance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _indexer.drive(_oi.getIndexerJoystick());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _indexer.drive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
