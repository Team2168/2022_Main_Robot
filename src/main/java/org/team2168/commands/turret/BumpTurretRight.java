// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpTurretRight extends CommandBase {
  /** Creates a new BumpTurretRight. */
  private Turret turret;
  private double setpoint = 5.0;

  /**
   * Bumps the turret one degree clockwise/to the right
   * @param turret the turret instance
   */
  public BumpTurretRight(Turret turret) {
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = turret.getSetpoint() + this.setpoint;
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
