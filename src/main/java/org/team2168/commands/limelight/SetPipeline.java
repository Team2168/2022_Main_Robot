// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.limelight;

import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPipeline extends CommandBase {
  /** Creates a new SetPipeline. */
  Limelight lime;
  int pipelineNum;
  boolean isPipelineSet = false;

  public SetPipeline(Limelight limelight, int pipeline) {
    lime = limelight;
    pipelineNum = pipeline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lime.setPipeline(pipelineNum);
    isPipelineSet = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isPipelineSet;
  }
}
