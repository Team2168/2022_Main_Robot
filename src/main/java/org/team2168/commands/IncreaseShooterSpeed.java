// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Shooter;

public class IncreaseShooterSpeed extends CommandBase {
  /** Creates a new DecreaseShooterSpeed. */
  private double speed; 
  private Shooter shooter;

  public IncreaseShooterSpeed(double k_Speed, Shooter k_Shooter) {
    k_Speed = speed;
    k_Shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speed < 1000){
      speed = speed + 50;
      shooter._motorRight.set(speed);
      shooter._motorLeft.set(speed); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (speed >= 1700) {
      return true;
    } else{
      return false;
    }
  }
}
