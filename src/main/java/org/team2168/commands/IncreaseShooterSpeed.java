// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.BackHood;

public class IncreaseShooterSpeed extends CommandBase {
  /** Creates a new DecreaseShooterSpeed. */
  private double Speed; 
  private Shooter Shooter;
  private BackHood BackHood;

  public IncreaseShooterSpeed(double k_Speed, Shooter k_Shooter, BackHood k_BackHood) {
    k_Speed = Speed;
    k_Shooter = Shooter;
    k_BackHood = BackHood; 
    addRequirements(Shooter, BackHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Speed < 100){
      Speed = Speed*2; //100 is just a placeholder number, the increasing increment is also just a placeholder 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished(); {
      interrupted = false;
    }
    interrupted = true; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
