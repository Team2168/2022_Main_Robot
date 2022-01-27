// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.verticalClimber;

import org.team2168.subsystems.VerticalClimber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReturnToZero extends CommandBase {
  /** Creates a new ReturnToZero. */
  VerticalClimber vClimber = VerticalClimber.getInstance();
  private double liftDescentVelocity = 1.0; // inches per second

  public ReturnToZero(VerticalClimber vClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vClimber.setSpeed(liftDescentVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vClimber.setSpeed(0.0);
    vClimber.setEncoderPosZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vClimber.isAtZeroPosition();
  }
}
