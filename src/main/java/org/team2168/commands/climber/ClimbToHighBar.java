// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.Constants.LiftPositions;
import org.team2168.commands.monkeybar.CheckMonkeyHookAttached;
import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.MonkeyBar;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToHighBar extends SequentialCommandGroup {
  /** Creates a new ClimbWithMonkeyBars. */

/**
 * This command allows for the robot to climb from one rung
 * to a higher one during the climbing sequence.
 * 
 * Should be used after robot initially climbs the mid bar.
 */
  public ClimbToHighBar(Climber climb, MonkeyBar monkey) {
    addCommands(
      new DriveClimberToPosition(climb, LiftPositions.LIFT_UNLOAD_TO_MBAR_INCHES),
      new CheckMonkeyHookAttached(monkey),
      new ExtendMonkeyBar(monkey),
      new DriveClimberToPosition(climb, LiftPositions.LIFT_EXTEND_BELOW_NEXT_BAR_INCHES),
      new WaitToExtendLiftWhileSwinging(LiftPositions.SAFE_HIGH_BAR_EXTEND_PITCH, LiftPositions.TOO_CLOSE_TO_SWING_APEX_PITCH),
      new DriveClimberToPosition(climb, LiftPositions.LIFT_ABOVE_BAR_FROM_AIR_INCHES),
      new RetractMonkeyBar(monkey),
      new CheckClimberHookAttached(climb, 10),
      new DriveClimberToPosition(climb, LiftPositions.LIFT_RETRACTION_INCHES));
  }
}
