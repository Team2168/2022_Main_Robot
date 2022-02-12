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
public class ClimbAndWaitForMonkey extends SequentialCommandGroup {
  /** Creates a new ClimbAndPassToMonkey. */
  Climber climb;

  public ClimbAndWaitForMonkey(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    climb = climber;
    addCommands(
      new SetPosition(climb, Constants.LiftPositions.LIFT_EXTENSION_INCHES),
      new CheckClimberHookAttached(climb),
      new SetPosition(climb, Constants.LiftPositions.LIFT_RETRACTION_INCHES));
  }
}
