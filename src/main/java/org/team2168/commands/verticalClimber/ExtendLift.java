// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.verticalClimber;

import org.team2168.subsystems.VerticalClimber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendLift extends CommandBase {
  /** Creates a new ExtendLift. */
  VerticalClimber vClimber = VerticalClimber.getInstance();
  private boolean isFullyExtended = false;
  private double liftAscentVelocity = -2.0; // inches per second
  private double maxExtensionTicks = -4096; // arbitrary max encoder value for when lift is fully extended

  public ExtendLift(VerticalClimber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vClimber.setSpeed(liftAscentVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vClimber.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (vClimber.getEncoderTicksMotor1() <= maxExtensionTicks || vClimber.getEncoderTicksMotor2() <= maxExtensionTicks) {
      isFullyExtended = true;
    }
    return isFullyExtended;
  }
}
