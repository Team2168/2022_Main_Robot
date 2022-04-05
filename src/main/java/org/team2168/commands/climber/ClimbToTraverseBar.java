// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.Constants;
import org.team2168.Constants.LiftPositions;
import org.team2168.commands.Sleep;
import org.team2168.commands.monkeybar.CheckMonkeyHookAttached;
import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.MonkeyBar;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimbToTraverseBar extends SequentialCommandGroup {

/**
 * This command allows for the robot to climb from one rung
 * to a higher one during the climbing sequence.
 * 
 * Should be used after robot initially climbs the high bar.
 */
  public ClimbToTraverseBar(Climber climb, MonkeyBar monkey) {
    addCommands(
      new DriveClimberToPosition(climb, Constants.LiftPositions.LIFT_ARRESTING_INCHES),
      // new Sleep().withTimeout(0.5), //stay connected with the climber bars to slow the swing down
      new WaitUntilLevelRobot(Constants.LiftPositions.LIFT_UNLOAD_TO_MBAR_PITCH),
      new DriveClimberToPosition(climb, LiftPositions.LIFT_EXTEND_BELOW_NEXT_BAR_INCHES),
      new CheckMonkeyHookAttached(monkey),
      new ExtendMonkeyBar(monkey),
      new WaitToExtendLiftWhileSwinging(LiftPositions.SAFE_TRAVERSE_BAR_EXTEND_PITCH, LiftPositions.TOO_CLOSE_TO_SWING_APEX_PITCH),
      new DriveClimberToPosition(climb, LiftPositions.LIFT_ABOVE_BAR_FROM_AIR_INCHES),
      new RetractMonkeyBar(monkey),
      new CheckClimberHookAttached(climb, 15),
      new DriveClimberToPosition(climb, Constants.LiftPositions.LIFT_RETRACT_TO_MINIMIZE_TRAVESE_SWING_INCHES),
      new WaitCommand(4.0),//swing low sweet chariot
      new DriveClimberToPosition(climb, Constants.LiftPositions.LIFT_RETRACTION_INCHES),
      new DriveClimberToPosition(climb, Constants.LiftPositions.LIFT_ARRESTING_INCHES)
    );
  }
}
