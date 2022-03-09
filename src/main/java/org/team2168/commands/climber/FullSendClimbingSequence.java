// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;
import org.team2168.subsystems.MonkeyBar;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullSendClimbingSequence extends SequentialCommandGroup {

  public FullSendClimbingSequence(Climber climber, MonkeyBar monkeyBar) {
    addCommands(
      new DriveClimberToZero(climber).withTimeout(2.0),
      new HangOnMidBar(climber),
      new ClimbToHighBar(climber, monkeyBar),
      new ClimbToTraverseBar(climber, monkeyBar)
    );
  }
}
