package org.team2168.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.Constants;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.ColorSensor;
import org.team2168.subsystems.Pooper;


public class QueueBalls extends CommandBase {
    Hopper hopper;
    Indexer indexer;
    ColorSensor colorSensor;
    Pooper pooper;

    public QueueBalls(Hopper hopper, Indexer indexer, ColorSensor colorSensor, Pooper pooper) {
        this.hopper = hopper;
        this.indexer = indexer;
        this.colorSensor = colorSensor;
        this.pooper = pooper;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(hopper, indexer, colorSensor);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hopper.driveHopper(Constants.MotorSpeeds.HOPPER_SPEED);
        if (colorSensor.isTeamColor()) {
            if (!indexer.isBallPresent())
                indexer.drive(Constants.MotorSpeeds.INDEXER_SPEED);
            else
                indexer.drive(0.0);
        }
        else {
           pooper.extend();
           pooper.retract(); 
        }
        

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        hopper.driveHopper(0.0);
        indexer.drive(0.0);
    }
}
