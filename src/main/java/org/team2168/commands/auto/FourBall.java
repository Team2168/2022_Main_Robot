// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team2168.commands.DriveHopperAndIndexer;
import org.team2168.commands.LowerAndRunIntake;
import org.team2168.commands.RetractAndStopIntake;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.TarmacLine;
import org.team2168.subsystems.*;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

public class FourBall extends SequentialCommandGroup {
    /**
     * Creates a new FourBall.
     */
    public FourBall(Drivetrain drivetrain, IntakeRaiseAndLower intakeRaiseAndLower, IntakeRoller intakeRoller, Hopper hopper, Indexer indexer, Hood hood, Shooter shooter) {

        Paths paths = Paths.getInstance();
        addCommands(
                new TarmacLine(hood, shooter),
                race (  // run group until path ends
                        new LowerAndRunIntake(intakeRaiseAndLower, intakeRoller, hopper, indexer),
                        PathUtil.getPathCommand(paths.path_4BALL_0, drivetrain, InitialPathState.DISCARDHEADING)
                ),
                new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller, hopper, indexer).withTimeout(0.1),
                new WaitForShooterAtSpeed(shooter).withTimeout(0.5),
                new DriveHopperAndIndexer(hopper, indexer).withTimeout(1),
                new WaitForShooterAtSpeed(shooter).withTimeout(0.5),
                new DriveHopperAndIndexer(hopper, indexer).withTimeout(1.5),


                race (
                        new LowerAndRunIntake(intakeRaiseAndLower, intakeRoller, hopper, indexer),
                        new SequentialCommandGroup(
                                PathUtil.getPathCommand(paths.path_4BALL_1, drivetrain, InitialPathState.PRESERVEODOMETRY),
                                PathUtil.getPathCommand(paths.path_4BALL_2, drivetrain, InitialPathState.PRESERVEODOMETRY),
                                PathUtil.getPathCommand(paths.path_4BALL_3, drivetrain, InitialPathState.PRESERVEODOMETRY)
                        )
                ),
                new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller, hopper, indexer).withTimeout(0.1),
                new WaitForShooterAtSpeed(shooter).withTimeout(0.5),
                new DriveHopperAndIndexer(hopper, indexer).withTimeout(1),
                new WaitForShooterAtSpeed(shooter).withTimeout(0.5),
                new DriveHopperAndIndexer(hopper, indexer).withTimeout(1.5),
                new DriveHopperAndIndexer(hopper, indexer).withTimeout(1.5)

        );
    }
}
