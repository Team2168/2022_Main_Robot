// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hopper;

import org.team2168.Constants.MotorSpeeds;
import org.team2168.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveHopperUntilBall extends CommandBase {
  private DoubleSupplier speed;
  private Hopper hopper;

  public DriveHopperUntilBall(Hopper hopper, DoubleSupplier speed) {
    this.hopper = hopper;
    this.speed = speed;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.driveHopper(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.driveHopper(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hopper.isBallPresent();
  }
}
