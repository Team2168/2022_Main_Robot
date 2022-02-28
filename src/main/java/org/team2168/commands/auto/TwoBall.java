package org.team2168.commands.auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team2168.commands.FireBalls;
import org.team2168.commands.QueueBallsForShotNoStop;
import org.team2168.commands.RetractAndStopIntake;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.commands.shooter.WaitForShooterAtSpeed;
import org.team2168.commands.shootingpositions.TarmacLine;
import org.team2168.subsystems.*;
import org.team2168.utils.PathUtil;

public class TwoBall extends SequentialCommandGroup {
    public TwoBall(
            Drivetrain drivetrain,
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
                        PathUtil.getPathCommand(paths.path_4BALL_0, drivetrain, PathUtil.InitialPathState.DISCARDHEADING)
                ),
                new RetractAndStopIntake(intakeRaiseAndLower, intakeRoller).withTimeout(0.1),

                parallel (
//                        new DriveWithLimelight(drivetrain, lime),
                        new WaitForShooterAtSpeed(shooter)
                ).withTimeout(7),
                new FireBalls(shooter, indexer, hopper),
                new WaitForShooterAtSpeed(shooter).withTimeout(0.5),
                new FireBalls(shooter, indexer, hopper)
        );
    }
}