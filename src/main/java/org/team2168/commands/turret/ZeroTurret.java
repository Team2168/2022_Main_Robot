// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.turret;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTurret extends CommandBase {
  /** Creates a new ZeroTurret. */
  private Turret turret;
  private double position;

  public ZeroTurret(Turret turret) {
    this.turret = turret;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = turret.getEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setRotationDegrees(-position);

    if(!turret.isTurretAtZero()) {
      if(turret.getEncoderPosition() > 0)
        turret.setVelocity(-0.5);
      else  
        turret.setVelocity(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    // if (!interrupted) {
    //   turret.zeroEncoder();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.isTurretAtZero();
  }
}
