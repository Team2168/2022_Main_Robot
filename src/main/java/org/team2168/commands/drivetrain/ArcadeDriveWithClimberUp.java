// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDriveWithClimberUp extends CommandBase {

  private Drivetrain dt;
  private DoubleSupplier speed;
  private DoubleSupplier turn;
  /** Creates a new ArcadeDriveWithClimberUp. */
  public ArcadeDriveWithClimberUp(Drivetrain drivetrain, DoubleSupplier s, DoubleSupplier t) {
    dt = drivetrain;
    s = speed;
    t = turn;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
