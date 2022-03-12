// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilLevelRobot extends CommandBase {
  /** Creates a new ExtendLiftWhileSwinging. */
  private Drivetrain dt;
  private double last_angle;
  private double safe_angle;
  private boolean safe_to_extend = false;
  private int acceptableLoops;
  private final static int DEFAULT_ACCEPTABLE_LOOPS = 50;
  private int withinThresholdLoops = 0;

  public WaitUntilLevelRobot(double angle, int acceptableLoops) {
    dt = Drivetrain.getInstance();
    safe_angle = angle;
    last_angle = safe_angle;
    this.acceptableLoops = acceptableLoops;
  }

  public WaitUntilLevelRobot(double angle) {
    this(angle, DEFAULT_ACCEPTABLE_LOOPS);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    withinThresholdLoops = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_angle = dt.getPitch() - dt.gyroOffset;

    if (current_angle <= safe_angle) {
      ++withinThresholdLoops;
    }
    else {
      withinThresholdLoops = 0;
    }

    safe_to_extend = (current_angle <= safe_angle) && (withinThresholdLoops >= acceptableLoops);

    last_angle = current_angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return safe_to_extend;
  }
}
