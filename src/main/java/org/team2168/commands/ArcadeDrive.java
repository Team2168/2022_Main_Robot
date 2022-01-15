// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {
  private Drivetrain dt;
  private DoubleSupplier speed;
  private DoubleSupplier turn;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier speed, DoubleSupplier turn) {
    dt = drivetrain;
    this.speed = speed;
    this.turn = turn;

    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.arcadeDrive(speed.getAsDouble(), turn.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
