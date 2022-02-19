// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourBall extends SequentialCommandGroup {
  /** Creates a new FourBall. */
  public FourBall(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Paths paths = Paths.getInstance();
    addCommands(
      PathUtil.getPathCommand(paths.path_4BALL_0, drivetrain, InitialPathState.DISCARDHEADING),
      PathUtil.getPathCommand(paths.path_4BALL_1, drivetrain, InitialPathState.PRESERVEODOMETRY),
      PathUtil.getPathCommand(paths.path_4BALL_2, drivetrain, InitialPathState.PRESERVEODOMETRY)
    );
  }
}
