// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveClimberWithJoystick extends CommandBase {

  private Climber climber;
  private DoubleSupplier speed;

  /** Creates a new DriverWithJoystick. */
  public DriveClimberWithJoystick(Climber climber, DoubleSupplier s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

    this.climber = climber;
    speed = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPercentOutput(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
