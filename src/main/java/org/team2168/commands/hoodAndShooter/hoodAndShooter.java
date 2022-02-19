// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hoodAndShooter;

import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.shooter.SetSpeed;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.Hood.HoodPosition;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class hoodAndShooter extends ParallelCommandGroup {
  /** Creates a new hoodAndShooter. */
  private Shooter shooter;
  private Hood hood;
  private HoodPosition hoodPosition;

  public hoodAndShooter(Shooter s, Hood h, HoodPosition hp) {
    shooter = s;
    hood = h;
    hoodPosition = hp;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    if (hoodPosition == HoodPosition.BACK_OF_TARMAC) {
      addCommands(
        new HoodToAngle(hood, HoodPosition.BACK_OF_TARMAC.position_degrees),
        new SetSpeed(shooter, 200.0) //TODO: change the rpm when able to test
      );
    }

    if (hoodPosition == HoodPosition.WHITE_LINE) {
      addCommands(
        new HoodToAngle(hood, HoodPosition.WHITE_LINE.position_degrees),
        new SetSpeed(shooter, 300.00) //TODO: change the rpm when able to test
      );
    }

    if (hoodPosition == HoodPosition.TERMINAL) {
      addCommands(
        new HoodToAngle(hood, HoodPosition.TERMINAL.position_degrees),
        new SetSpeed(shooter, 400.00) //TODO: change the rpm when able to test
      );
    }
  }
}
