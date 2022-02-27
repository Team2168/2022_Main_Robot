package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.hopper.DriveHopperWithPercentOutput;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;

public class RetractAndStopIntake extends ParallelCommandGroup {
    public RetractAndStopIntake(IntakeRaiseAndLower intakeRaiseAndLower, IntakeRoller intakeRoller, Hopper hopper, Indexer indexer) {
        addCommands(
                new SetIntakeSpeed(intakeRoller, 0.0),
                new IntakeRaise(intakeRaiseAndLower),
                new DriveHopperWithPercentOutput(hopper, () -> 0.0),
                new DriveIndexer(indexer, () -> 0.0)
        );
    }
}