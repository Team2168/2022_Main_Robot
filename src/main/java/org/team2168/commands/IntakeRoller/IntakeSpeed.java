// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.IntakeRoller;

import org.team2168.subsystems.IntakeRoller;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class IntakeSpeed extends CommandBase {
 
  private IntakeRoller iRoller;
  private double speedValueForIntakeSpeed;
  // speedValueForIntakeSpeeed sets the speed value between -1 and 1 for the talonFX spin amount to intake balls in using the force
  // the spin
  public IntakeSpeed(IntakeRoller iRoller,double speedValueForIntakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    iRoller = IntakeRoller.getInstance();
    addRequirements(iRoller);
    this.speedValueForIntakeSpeed = speedValueForIntakeSpeed;
  }
  
  // Called when the command is initially scheduled.
  
  
  public void initialize() {
    
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
   iRoller.setRollerSpeed(speedValueForIntakeSpeed);
  
   
  }
   
   // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    iRoller.resetIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
