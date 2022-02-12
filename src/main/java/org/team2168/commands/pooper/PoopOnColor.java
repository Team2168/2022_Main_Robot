// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.pooper;

import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Pooper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoopOnColor extends CommandBase {

  /** Creates a new PoopOnColor. */
  public PoopOnColor(ColorSensor colorSensor, Pooper pooper, DriverStation ds) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.getAlliance();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
