// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpTurretLeft extends CommandBase {
  /** Creates a new BumpTurretLeft. */
  private Turret turret;

  /**
   * Bumps the turret one degree counterclockwise/to the left
   * @param turret the turret instance
   */
  public BumpTurretLeft(Turret turret) {
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = turret.getSetpoint() - 5.0;
    turret.setRotationDegrees(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
