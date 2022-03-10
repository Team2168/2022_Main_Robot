package org.team2168.commands.auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team2168.commands.drivetrain.DriveXDistance;
import org.team2168.subsystems.Drivetrain;

public class Drive3Feet extends SequentialCommandGroup {
    public Drive3Feet(Drivetrain drivetrain) {
        addCommands(
                sequence(new DriveXDistance(drivetrain, 3.28))
        );
    }
}