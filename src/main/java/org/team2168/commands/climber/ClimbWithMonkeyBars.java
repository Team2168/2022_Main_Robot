// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.Constants;
import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.MonkeyBar;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbWithMonkeyBars extends SequentialCommandGroup {
  /** Creates a new ClimbWithMonkeyBars. */
  Climber climb;
  MonkeyBar monkey;

  public ClimbWithMonkeyBars(Climber climber, MonkeyBar monkeyBar) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    climb = climber;
    monkeyBar = monkey;
    addCommands(new ExtendMonkeyBar(monkeyBar),
    // check if monkey bar hook limit switch is closed with command here
    new SetPosition(climber, Constants.LiftPositions.LIFT_EXTENSION_INCHES),
    // check if climber hook limit switch is closed with command here
    new RetractMonkeyBar(monkeyBar),
    new SetPosition(climber, Constants.LiftPositions.LIFT_RETRACTION_INCHES));
  }
}
