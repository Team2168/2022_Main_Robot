// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.IntakeRoller;

import org.team2168.subsystems.IntakeRoller;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class SetIntakeSpeed extends CommandBase {
 
  private IntakeRoller iRoller;
  private double speedValueForIntakeSpeed;

  /**
   * 
   * @param iRoller
   * @param speedValueForIntakeSpeed speed of intake roller in %output mode
   */
  public SetIntakeSpeed(IntakeRoller iRoller, double speedValueForIntakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.iRoller = iRoller;
    this.speedValueForIntakeSpeed = speedValueForIntakeSpeed;
    addRequirements(iRoller);
  }
  
  
  // Called when the command is initially scheduled.
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    iRoller.setRollerSpeed(speedValueForIntakeSpeed);  
  }
   
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    iRoller.setRollerSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
