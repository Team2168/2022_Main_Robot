package org.team2168.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team2168.commands.hood.DriveHoodToAngle;
import org.team2168.subsystems.Hood;

public class StowEverything extends ParallelCommandGroup {
    public StowEverything(Hood hood) {
        addCommands(
                new DriveHoodToAngle(hood, Hood.HoodPosition.ZERO.position_degrees)
        );
    }
}