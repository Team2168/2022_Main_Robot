// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateTurretNoFinish extends CommandBase {
  /** Creates a new RotateTurret. */
  private Turret turret;
  private double targetPositionDegrees;

  /**
   * Rotates the turret to a desired absolute heading
   * @param t the turret subsystem to be used
   * @param targetPosition the desired absolute position of the turret (degrees)
   */
  public RotateTurretNoFinish(Turret t, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = t;
    targetPositionDegrees = targetPosition;

    addRequirements(t);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setRotationDegrees(targetPositionDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
