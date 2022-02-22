package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team2168.Constants;
import org.team2168.commands.IntakeRoller.DriveIntakeToSpeed;
import org.team2168.commands.intakeraiseandlower.ExtendIntake;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;

public class ExtendAndRunIntake extends ParallelCommandGroup {
    public ExtendAndRunIntake(IntakeRaiseAndLower intakeRaiseAndLower, IntakeRoller intakeRoller) {
        addCommands(
                new ExtendIntake(intakeRaiseAndLower),
                new DriveIntakeToSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED)
        );
    }
}