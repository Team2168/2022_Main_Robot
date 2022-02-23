package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team2168.Constants;
import org.team2168.commands.IntakeRoller.SetIntakeSpeed;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;

public class LowerAndRunIntake extends ParallelCommandGroup {
    public LowerAndRunIntake(IntakeRaiseAndLower intakeRaiseAndLower, IntakeRoller intakeRoller, Hopper hopper, Indexer indexer) {
        addCommands(
                new IntakeLower(intakeRaiseAndLower),
                new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED),
                new QueueBalls(hopper, indexer)
        );
    }
}