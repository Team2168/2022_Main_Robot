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
   *  The amount of degrees the turret should rotate
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
    turret.setVelocity(10.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    //Checks if the current position of the turret is where it should be (the starting postition plus the desired amount of degrees)
    // TODO: See what in here is not working correctly and is preventing this method from ever returning true
    return (turret.getEncoderPosition() >= (position + (degrees/360 * 2048)));
  }
}
