// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.StopMechanisms;
import org.team2168.commands.WaitUntilFireBalls;
import org.team2168.commands.hood.HoodToAngle;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.commands.limelight.SetPipeline;
import org.team2168.commands.shooter.SetShooterSpeed;
import org.team2168.commands.shootingpositions.ShootBasedOnDistance;
import org.team2168.commands.turret.DriveTurretWithLimelight;
import org.team2168.commands.turret.RotateTurret;
import org.team2168.commands.turret.StopTurret;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hood;
import org.team2168.subsystems.Hood.HoodPosition;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Pooper;
import org.team2168.subsystems.Shooter;
import org.team2168.subsystems.Shooter.ShooterRPM;
import org.team2168.subsystems.Turret;
import org.team2168.utils.PathUtil;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftSideThreeAuto extends SequentialCommandGroup {

  public LeftSideThreeAuto(
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
      Paths path = Paths.getInstance();

    addCommands(
     // Resets the turret and prevents shooter from waiting to reach speed, precautions before starting auto
        new RotateTurret(turret, 0.0).withTimeout(0.2),
        new InstantCommand(() -> shooter.setWaitForShooterAtSpeed(false)),

        //The Limelight Race command will be active all the time to continously track the hub or use vision processing

        race(
            new DriveTurretWithLimelight(turret, limelight),
            new ShootBasedOnDistance(shooter, hood, limelight),
              // First sequence sets the shooter and hood for shots close to the tarmac line
              // The robot moves backwards to collect a ball, the intake lower method lowers the intake
              // to allow the QueueBallsForShotNoStop Method to collect the ball and prepare the shot
            sequence(
                new IntakeLower(intakeRaiseAndLower),
                  race(
                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                    PathUtil.getPathCommand(path.path_TwoBallLeft, drivetrain,
                        PathUtil.InitialPathState.DISCARDHEADING)
                  ))),
                  // The Robot moves back to the edge of the tarmac and shoots the shot
                    sequence(

                        new StopMechanisms(hopper, indexer, intakeRoller),
                        //race(
                            //PathUtil.getPathCommand(path.path_ReverseTwoBallLeft, drivetrain,
                               // PathUtil.InitialPathState.PRESERVEODOMETRY))),

                    sequence(
                        new WaitUntilFireBalls(shooter, limelight),
                        new FireBalls(shooter, indexer, hopper),
                        new FireBalls(shooter, indexer, hopper),

                        new StopMechanisms(hopper, indexer, intakeRoller)
                    ),
                    // The Intake is Raised whilst the robot moves around the hangar to the terminal assembly (loading cargo station) to collect a ball
                        sequence(
                            new IntakeLower(intakeRaiseAndLower),

                            race(
                                new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                PathUtil.getPathCommand(path.path_LineThreeSetupAuto, drivetrain,
                                    PathUtil.InitialPathState.PRESERVEODOMETRY))),
                             
                                    
                            sequence(

                              parallel(
                                new StopMechanisms(hopper, indexer, intakeRoller),
                                new IntakeRaise(intakeRaiseAndLower)
                              ),
                                race(
                                    PathUtil.getPathCommand(path.path_ReversedThreeSetupAuto, drivetrain,
                                        PathUtil.InitialPathState.PRESERVEODOMETRY))),
                                      // returns back to the tarmac line to setup shot sequence
                                sequence(
                                   
                                        race(
                                            new SetPipeline(limelight, Limelight.PIPELINE_TARMAC_LINE
                                            )),

                                    new StopMechanisms(hopper, indexer, intakeRoller, drivetrain),
                                    new WaitUntilFireBalls(shooter, limelight),
                                    new FireBalls(shooter, indexer, hopper),
                                    new FireBalls(shooter, indexer, hopper)),

                                parallel(
                                    new SetShooterSpeed(shooter, ShooterRPM.STOP),
                                    new StopTurret(turret)

                                )));
  }
}
