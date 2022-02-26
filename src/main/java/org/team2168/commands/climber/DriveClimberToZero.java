// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveClimberToZero extends CommandBase {
  /** Creates a new ReturnToZero. */
  Climber climber;
  // private static final double LIFT_DESCENT_VELOCITY_IPS = -1.0; // inches per second
  private static final double LIFT_DESCENT_PERCENT_OUTPUT = -0.05;

  public DriveClimberToZero(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPercentOutput(LIFT_DESCENT_PERCENT_OUTPUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      //Don't zero if we didn't get to the sensor
      climber.setPercentOutput(0.0);
      climber.setEncoderPosZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isAtZeroPosition();
  }
}
