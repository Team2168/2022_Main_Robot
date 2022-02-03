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
    addCommands(new ExtendLift(climb), // climbing the mid bar
    new WaitCommand(0.5), // gives driver some additional time to drive robot so that it presses against mid bar
    new RetractLift(climb),
    new ExtendMonkeyBar(monkey),
    new ExtendLift(climb), // climbing the high bar
    new RetractMonkeyBar(monkey),
    new RetractLift(climb),
    new ExtendMonkeyBar(monkey),
    new ExtendLift(climb), // climbing the transversal bar
    new RetractMonkeyBar(monkey),
    new RetractLift(climb));
  }
}
