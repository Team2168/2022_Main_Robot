package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.intakeraiseandlower.IntakeRaise;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;

public class RetractAndStopIntake extends ParallelCommandGroup {
    public RetractAndStopIntake(IntakeRaiseAndLower intakeRaiseAndLower, IntakeRoller intakeRoller) {
        addCommands(
                new SetIntakeSpeed(intakeRoller, 0.0),
                new IntakeRaise(intakeRaiseAndLower)
        );
        
    }
}