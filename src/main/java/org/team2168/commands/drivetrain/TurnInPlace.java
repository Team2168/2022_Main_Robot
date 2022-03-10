// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnInPlace extends CommandBase {
 public org.team2168.subsystems.Drivetrain turnInPlace;
 public boolean whichDirection;
 public boolean right;
 public boolean left;
  public TurnInPlace(org.team2168.subsystems.Drivetrain turnInPlace, boolean whichDirection) {
    this.whichDirection = whichDirection;
    this.turnInPlace = turnInPlace;
    addRequirements(turnInPlace);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(whichDirection == right){
   turnInPlace.tankDrive(0, 1);
    }
    else if(whichDirection == left){
      turnInPlace.tankDrive(1, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnInPlace.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
