// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTurret extends CommandBase {
  /** Creates a new ZeroTurret. */
  private Turret motor;
  private double position;

  public ZeroTurret(Turret motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motor = motor;
    position = motor.getEncoderPosition();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if the turret is facing the left of its 0, it will turn right and vice versa
    if (position < 0) {
      motor.setSpeed(0.5);
    }
    else {
      motor.setSpeed(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.zeroEncoder();
    motor.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motor.isTurretAtZero();
  }
}
