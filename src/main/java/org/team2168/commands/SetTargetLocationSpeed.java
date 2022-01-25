// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.Shooter; 
import org.team2168.subsystems.Shooter.FiringLocation;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetTargetLocationSpeed extends CommandBase {
  /** Creates a new SetTargetLocationSpeed. */

  private Shooter shooter;
  public double targetVelocity;
  public FiringLocation k_Location; //why is the variable type not working


  public SetTargetLocationSpeed(double k_setPoint, double k_targetVelocity) {
    shooter = shooter.getInstance(); //im not sure what to get
    addRequirements(shooter);
    k_targetVelocity = k_setPoint;
  }

  public SetTargetLocationSpeed(FiringLocation d_setPoint, FiringLocation d_Location, double d_targetVelocity) {
    shooter = shooter.getInstance();
    addRequirements(shooter);
    k_Location = d_setPoint;
    d_targetVelocity = d_setPoint.getSpeed();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(k_Location != null){
      shooter.setFiringLocation(targetVelocity); //new method created in shooter.java that doesn't work
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(targetVelocity);
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
