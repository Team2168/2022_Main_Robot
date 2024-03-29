package org.team2168.commands.pooper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Pooper;


public class PoopOnColor extends CommandBase {
    ColorSensor colorSensor;
    Pooper pooper;

    public PoopOnColor(ColorSensor colorSensor, Pooper pooper) {
        this.colorSensor = colorSensor;
        this.pooper = pooper;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(pooper);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (!colorSensor.isDataStale() && !colorSensor.isTeamColor()){
                pooper.extend();
        }
        else pooper.retract();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pooper.retract();

    }
}
