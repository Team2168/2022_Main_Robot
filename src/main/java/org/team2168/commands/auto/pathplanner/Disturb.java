// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.Constants;
import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.drivetrain.TankDrive;
import org.team2168.commands.hopper.DriveHopperUntilBall;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.limelight.WaitForLimelightInPosition;
import org.team2168.commands.pooper.PoopOnColor;
import org.team2168.commands.pooper.PooperUnpoop;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.ShootBasedOnDistance;
import org.team2168.commands.turret.DriveTurretWithLimelight;
import org.team2168.commands.turret.RotateTurret;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Pooper;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.Turret;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Disturb extends SequentialCommandGroup {
  /** Creates a new Disturb. */
  public Disturb(
      Drivetrain drivetrain,
      IntakeRaiseAndLower intakeRaiseAndLower,
      IntakeRoller intakeRoller,
      Hopper hopper,
      Indexer indexer,
      Hood hood,
      Shooter shooter,
      Turret turret,
      Pooper pooper,
      ColorSensor colorSensor,
      Limelight limelight) {

    Paths paths = Paths.getInstance();

    addCommands(
        new RotateTurret(turret, 0.0).withTimeout(0.1),
        new InstantCommand(() -> shooter.setWaitForShooterAtSpeed(false)),
        new PooperUnpoop(pooper),

        race( 
          new DriveTurretWithLimelight(turret, limelight),
          new ShootBasedOnDistance(shooter, hood, limelight),
          sequence( //shoots preloaded ball, collects and poops opposite's ball
            new IntakeLower(intakeRaiseAndLower),
            race(
              PathUtil.getPathCommand(paths.path_Disturb_1, drivetrain, InitialPathState.DISCARDHEADING),
              new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller)),

            PathUtil.getPathCommand(paths.path_Disturb_2, drivetrain, InitialPathState.PRESERVEODOMETRY),
            new PoopOnColor(colorSensor, pooper, hopper),
            new PooperUnpoop(pooper),

            //Collects and shoots another ball
            race(
              new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
              PathUtil.getPathCommand(paths.path_Disturb_3, drivetrain, InitialPathState.PRESERVEODOMETRY)),
            parallel(
              new SetIntakeSpeed(intakeRoller, 0.0),
              new DriveHopperWithPercentOutput(hopper, () -> 0.0),
              new DriveIndexer(indexer, () -> 0.0),
              new TankDrive(drivetrain, () -> 0.0, () -> 0.0)).withTimeout(0.1),
            parallel(
              new WaitForShooterAtSpeed(shooter),
              new WaitForLimelightInPosition(limelight)).withTimeout(2.0),

            new FireBalls(shooter, indexer, hopper),
            parallel(
              new WaitForShooterAtSpeed(shooter),
              new WaitForLimelightInPosition(limelight)).withTimeout(1.0),
            new FireBalls(shooter, indexer, hopper),

            // Collects and poops opponent's ball
            parallel(
              PathUtil.getPathCommand(paths.path_Disturb_4, drivetrain, InitialPathState.PRESERVEODOMETRY),
              race(
                new DriveHopperUntilBall(hopper, () -> Constants.MotorSpeeds.HOPPER_SPEED),
                new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED))),

            parallel(
              new SetIntakeSpeed(intakeRoller, 0.0),
              new DriveHopperWithPercentOutput(hopper, () -> 0.0),
              new DriveIndexer(indexer, () -> 0.0),
              new TankDrive(drivetrain, () -> 0.0, () -> 0.0)).withTimeout(0.1),

            PathUtil.getPathCommand(paths.path_Disturb_5, drivetrain, InitialPathState.PRESERVEODOMETRY),
            new PoopOnColor(colorSensor, pooper, hopper)
            )
    ));
  }
}
