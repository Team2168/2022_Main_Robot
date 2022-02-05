// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopIntakeMotor extends CommandBase {
  private IntakeRoller stopIntakeMotorOne;
  private final double resetIntakeValue = 0;
 
  public StopIntakeMotor() {
stopIntakeMotorOne = new IntakeRoller(false);
addRequirements(stopIntakeMotorOne);
  }

  // Called when the command is initially scheduled.
 
  public void resetMotor(){
   stopIntakeMotorOne.resetIntakeMotor(resetIntakeValue);
 }

  @Override 
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    resetMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
