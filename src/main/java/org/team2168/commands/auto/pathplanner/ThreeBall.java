package org.team2168.commands.auto.pathplanner;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team2168.Constants;
import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.limelight.WaitForLimelightInPosition;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.ShootBasedOnDistance;
import org.team2168.commands.turret.DriveTurretWithLimelight;
import org.team2168.commands.turret.RotateTurret;
import org.team2168.subsystems.*;
import org.team2168.utils.PathUtil;

public class ThreeBall extends SequentialCommandGroup {
    public ThreeBall(
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
            Limelight lime) {

        Paths paths = Paths.getInstance();
        addCommands(
                new RotateTurret(turret, 0.0).withTimeout(0.2),
                new InstantCommand(() -> shooter.setWaitForShooterAtSpeed(false)),
                parallel(
                        new DriveTurretWithLimelight(turret, lime),
                        new ShootBasedOnDistance(shooter, hood, lime),
                        sequence(
                            new IntakeLower(intakeRaiseAndLower),
                            race (  // run group until path ends
                                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                    PathUtil.getPathCommand(paths.path_3BALL_0, drivetrain, PathUtil.InitialPathState.DISCARDHEADING)
                            ),
                            new DriveIndexer(indexer, () -> 0.0).withTimeout(0.1),

                            new WaitForLimelightInPosition(lime),
                            new FireBalls(shooter, indexer, hopper),
                            new WaitForShooterAtSpeed(shooter),
                            new FireBalls(shooter, indexer, hopper),


                            new IntakeLower(intakeRaiseAndLower),
                            race (
                                    new QueueBallsForShotNoStop(hopper, indexer, pooper, colorSensor, intakeRoller),
                                    PathUtil.getPathCommand(paths.path_3BALL_1, drivetrain, PathUtil.InitialPathState.PRESERVEODOMETRY)
                                ),

                            race (
                                    new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED),
                                    sequence (
                                            new WaitForLimelightInPosition(lime),
                                            new FireBalls(shooter, indexer, hopper)
                                    )
                            ),
                            
                            race (
                                    new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED),
                                    new FireBalls(shooter, indexer, hopper)
                                )
                )
                )
        );
    }
}