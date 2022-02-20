// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hopper;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveHopperWithVelocity extends CommandBase {

  private Hopper hopper;
  private DoubleSupplier velocity;

  /** Creates a new DriveHopperWithVelocity. */
  public DriveHopperWithVelocity(Hopper hopper, DoubleSupplier velocity) {
    this.hopper = hopper;
    this.velocity = velocity;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopper.driveHopperVelocity(velocity.getAsDouble());
    System.out.println(hopper.getEncoderPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  hopper.driveHopperVelocity(0.0);
  System.out.println("stopped");
  hopper.zeroEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
