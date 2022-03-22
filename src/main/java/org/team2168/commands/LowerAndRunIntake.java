package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.Constants;
import org.team2168.commands.intakeroller.SetIntakeSpeed;
import org.team2168.commands.intakeraiseandlower.IntakeLower;
import org.team2168.subsystems.IntakeRaiseAndLower;
import org.team2168.subsystems.IntakeRoller;

public class LowerAndRunIntake extends ParallelCommandGroup {
    public LowerAndRunIntake(IntakeRaiseAndLower intakeRaiseAndLower, IntakeRoller intakeRoller) {
        addCommands(
                new IntakeLower(intakeRaiseAndLower),
                new SetIntakeSpeed(intakeRoller, Constants.MotorSpeeds.INTAKE_SPEED)
                
        );
    }
}