// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.commands.monkeybar.ExtendMonkeyBar;
import org.team2168.commands.monkeybar.RetractMonkeyBar;
import org.team2168.subsystems.Climber;
import org.team2168.subsystems.MonkeyBar;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbingSequence extends SequentialCommandGroup {
  /** Creates a new ClimbingSequence. */
  Climber climb = Climber.getInstance();
  MonkeyBar monkey = MonkeyBar.getInstance();
  public ClimbingSequence() {
    addCommands(
      new HangOnMidBar(climb),
      new ClimbWithMonkeyBars(climb, monkey),
      new ClimbWithMonkeyBars(climb, monkey));
  }
}
