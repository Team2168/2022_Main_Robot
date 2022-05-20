package org.team2168.commands.pooper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Pooper;


public class AutoPoopOnColor extends CommandBase {
    Hopper hopper;
    ColorSensor colorSensor;
    Pooper pooper;

    public AutoPoopOnColor(ColorSensor colorSensor, Pooper pooper, Hopper hopper) {
        this.colorSensor = colorSensor;
        this.pooper = pooper;
        this.hopper = hopper;
        
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
        return !hopper.isBallPresent();
    }

    @Override
    public void end(boolean interrupted) {
        pooper.retract();

    }
}