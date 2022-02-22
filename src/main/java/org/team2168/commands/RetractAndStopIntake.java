package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.IntakeRoller.DriveIntakeToSpeed;
import org.team2168.commands.intakeraiseandlower.RetractIntake;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;

public class RetractAndStopIntake extends ParallelCommandGroup {
    public RetractAndStopIntake(IntakeRaiseAndLower intakeRaiseAndLower, IntakeRoller intakeRoller) {
        addCommands(
                new DriveIntakeToSpeed(intakeRoller, 0.0),
                new RetractIntake(intakeRaiseAndLower)
        );
    }
}