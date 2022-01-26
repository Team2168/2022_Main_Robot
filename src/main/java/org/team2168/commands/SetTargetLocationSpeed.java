// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.Shooter; 

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetTargetLocationSpeed extends CommandBase {
  /** Creates a new SetTargetLocationSpeed. */

  private Shooter shooter;
  public double targetVelocity;
  public double[] locationCoords; 


  public SetTargetLocationSpeed(double k_setPoint, double k_targetVelocity) {
    shooter = shooter.getInstance(); //method not working
    addRequirements(shooter);
    k_targetVelocity = k_setPoint;
  }

  public SetTargetLocationSpeed(double[] d_setPoint, double x_targetVelocity, double y_targetVelocity, double[] totalVelocity) {
    shooter = shooter.getInstance();
    addRequirements(shooter);
    locationCoords = d_setPoint;
    x_targetVelocity = d_setPoint[0].getSpeed();
    y_targetVelocity = d_setPoint[1].getSpeed();
    totalVelocity = {x_targetVelocity, y_targetVelocity}; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(locationCoords != null){
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
