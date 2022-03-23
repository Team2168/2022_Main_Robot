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
  private double too_close_to_apex_angle;
  private boolean safe_to_extend = false;

  /**
   * 
   * @param safeAngle pitch angle above which we should try to extend the lift fully, if we're swinging away from the bar
   * @param apexAngle angle above which we shouldn't attempt to extend (e.g. we're too close to the swing apex and will soon start traveling back towards the next bar)
   */
  public WaitToExtendLiftWhileSwinging(double safeAngle, double apexAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = Drivetrain.getInstance();
    too_close_to_apex_angle = apexAngle;
    safe_angle = safeAngle;
    last_angle = safe_angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_angle = dt.getPitch();
    boolean swinging_away_from_bar = last_angle < current_angle;

    safe_to_extend = (current_angle >= safe_angle) && swinging_away_from_bar
                      && current_angle < too_close_to_apex_angle;

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
