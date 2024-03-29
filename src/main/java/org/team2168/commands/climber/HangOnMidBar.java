// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.Constants;
import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangOnMidBar extends SequentialCommandGroup {
  /** Creates a new ClimbAndPassToMonkey. */

/**
 * This command allows the robot to hang onto and climb the mid rung.
 */
  public HangOnMidBar(Climber climb) {
    addCommands(
      new DriveClimberToPosition(climb, Constants.LiftPositions.LIFT_ABOVE_BAR_FROM_GROUND_INCHES),
      new CheckClimberHookAttached(climb, 10), // wait for small # of loop iterations to verify we've settled against the bar before  beginning sequence
      new DriveClimberToPosition(climb, Constants.LiftPositions.LIFT_ZERO_INCHES));
  }
}
