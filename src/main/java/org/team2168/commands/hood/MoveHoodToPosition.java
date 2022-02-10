// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.hood;

import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Hood.HoodPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveHoodToPosition extends SequentialCommandGroup {
  /** Creates a new MoveHoodToPosition. */
  HoodPosition hp;
  Hood h;
  public MoveHoodToPosition(Hood hood, HoodPosition hoodPosition) {
    h = hood;
    hp = hoodPosition;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      if (hp == HoodPosition.TEST) {
         addCommands(new HoodToAngle(h, 30));
      }

      if (hp == HoodPosition.TEST1) {
         addCommands(new HoodToAngle(h, 45));
      }

      if (hp == HoodPosition.TEST2) {
         addCommands(new HoodToAngle(h, 60));
      }
      
      if (hp == HoodPosition.TEST3) {
         addCommands(new HoodToAngle(h, 90));
      }
    
  }
}
