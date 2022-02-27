// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.Constants.MotorSpeeds;
import org.team2168.commands.hopper.DriveHopperUntilBall;
import org.team2168.commands.indexer.DriveIndexerUntilBall;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.pooper.PoopOnColor;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.Pooper;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class QueueBallForShot extends SequentialCommandGroup {
  Hopper hopper = Hopper.getInstance();
  Pooper pooper = Pooper.getInstance();
  ColorSensor colorSensor= ColorSensor.getInstance();
  Indexer indexer = Indexer.getInstance();
  IntakeRoller intake = IntakeRoller.getInstance();

  /** Creates a new SendToFire. */
  public QueueBallForShot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      race(new SetIntakeSpeed(intake, MotorSpeeds.INTAKE_SPEED),
           new DriveHopperUntilBall(hopper, ()->MotorSpeeds.HOPPER_SPEED)
      ),   
      new PoopOnColor(colorSensor, pooper).withTimeout(0.5),
      new DriveIndexerUntilBall(indexer, ()->MotorSpeeds.INDEXER_SPEED).withTimeout(2.0)
    );

  }
}
