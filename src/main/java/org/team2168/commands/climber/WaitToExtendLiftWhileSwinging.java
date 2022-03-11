// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitToExtendLiftWhileSwinging extends CommandBase {
  private Drivetrain dt;
  private double last_angle;
  private double safe_angle;
  private boolean safe_to_extend = false;

  /** Creates a new ExtendLiftWhileSwinging. */
  public WaitToExtendLiftWhileSwinging(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = Drivetrain.getInstance();
    safe_angle = angle;
    last_angle = safe_angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_angle = dt.getPitch() - dt.gyroOffset;
    boolean swinging_away_from_bar = last_angle < current_angle;

    safe_to_extend = (current_angle >= safe_angle) && swinging_away_from_bar;

    last_angle = current_angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //finished when angle is safe to extend and we are swinging away from the bar.
    return safe_to_extend;
  }
}
