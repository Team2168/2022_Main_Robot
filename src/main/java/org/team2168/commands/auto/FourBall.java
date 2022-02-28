// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team2168.commands.*;
import org.team2168.commands.IntakeRoller.SetIntakeSpeed;
import org.team2168.commands.drivetrain.DriveWithLimelight;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.TarmacLine;
import org.team2168.subsystems.*;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

public class FourBall extends SequentialCommandGroup {
    /**
     * Creates a new FourBall.
     */
    public FourBall(Drivetrain drivetrain,
                    IntakeRaiseAndLower intakeRaiseAndLower,
                    IntakeRoller intakeRoller,
                    Hopper hopper,
                    Indexer indexer,
                    Hood hood,
                    Shooter shooter,
                    Pooper pooper,
                    ColorSensor colorSensor,
                    Limelight lime) {

        Paths paths = Paths.getInstance();
        addCommands(
                new TarmacLine(hood, shooter, lime).withTimeout(0.2),
                new IntakeLower(intakeRaiseAndLower),
                race (  // run group until path ends
                        new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                        PathUtil.getPathCommand(paths.path_4BALL_0, drivetrain, InitialPathState.DISCARDHEADING)
                ),
                new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller).withTimeout(0.1),

                parallel (
//                        new DriveWithLimelight(drivetrain, lime),
                        new WaitForShooterAtSpeed(shooter)
                ).withTimeout(30),
                new FireBalls(shooter, indexer, hopper),
                new WaitForShooterAtSpeed(shooter).withTimeout(0.5),
                new FireBalls(shooter, indexer, hopper),



                new IntakeLower(intakeRaiseAndLower),
                race (
                        new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                        PathUtil.getPathCommand(paths.path_4BALL_AIO, drivetrain, InitialPathState.PRESERVEODOMETRY)
                ),
                new IntakeRaise(intakeRaiseAndLower),
                new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
                PathUtil.getPathCommand(paths.path_4BALL_3, drivetrain, InitialPathState.PRESERVEODOMETRY),

                parallel (
//                new DriveWithLimelight(drivetrain, lime),
                new WaitForShooterAtSpeed(shooter)
                ).withTimeout(30),
                new FireBalls(shooter, indexer, hopper),
                new WaitForShooterAtSpeed(shooter).withTimeout(0.5),
                new FireBalls(shooter, indexer, hopper)
        );
    }
}
