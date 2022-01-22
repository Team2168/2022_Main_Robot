// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateTurret extends CommandBase {
  /** Creates a new RotateTurret. */
  private Turret turret;
  private double degrees;
  private double position;

  /**
   * Rotates the turret
   * @param t 
   *  The turret subsystem to be used
   * @param d 
   *  The amount the turret should rotate, -1 to 1
   */

  public RotateTurret(Turret t, double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = t;
    degrees = d;

    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = turret.getEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setRotationDegrees(degrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Checks if the current position of the turret is where it should be (the starting postition plus the desired amount of degrees)
    return (turret.getEncoderPosition() == position + degrees);
  }
}