// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTurretWithJoystick extends CommandBase {
  /** Creates a new DriveTurret. */
  private Turret turret;
  private DoubleSupplier speed;

  public DriveTurretWithJoystick(Turret t, DoubleSupplier s) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = t;
    speed = s;

    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.drive(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.drive(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
