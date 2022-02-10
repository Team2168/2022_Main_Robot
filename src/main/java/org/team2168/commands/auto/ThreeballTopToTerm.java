// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeballTopToTerm extends SequentialCommandGroup {
  /** Creates a new ThreeballTopToTerm. */
  public ThreeballTopToTerm(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      PathUtil.getPathCommand("2BALL_TOP_TO_TERM_0", drivetrain),
      PathUtil.getPathCommand("3BALL_TOP_TO_TERM_1", drivetrain, InitialPathState.PRESERVEODOMETRY),
      PathUtil.getPathCommand("2BALL_MID_TO_TERM_1", drivetrain, InitialPathState.PRESERVEODOMETRY)
    );
  }
}
