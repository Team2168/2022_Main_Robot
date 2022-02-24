// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.IntakeRoller;

import org.team2168.subsystems.IntakeRoller;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class DriveIntakeWithPercentOutput extends CommandBase {
 
  private IntakeRoller intakeRoller;
  private double speed;
  // speedValueForIntakeSpeed sets the speed value between -1 and 1 for the talonFX spin amount to intake balls in using the force
  // the spin
  public DriveIntakeWithPercentOutput(IntakeRoller intakeRoller, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeRoller = intakeRoller;
    this.speed = speed;
    addRequirements(intakeRoller);
  }
  
  
  // Called when the command is initially scheduled.
  
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intakeRoller.setRollerSpeed(speed);
  }
   
   // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intakeRoller.setRollerSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
