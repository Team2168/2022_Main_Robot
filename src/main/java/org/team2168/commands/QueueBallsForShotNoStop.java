package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.Constants;
import org.team2168.subsystems.*;


public class QueueBallsForShotNoStop extends CommandBase {
    Hopper hopper;
    Indexer indexer;
    Pooper pooper;
    ColorSensor colorSensor;
    IntakeRoller intakeRoller;

    public QueueBallsForShotNoStop(Hopper hopper, Indexer indexer, Pooper pooper, ColorSensor colorSensor, IntakeRoller intakeRoller) {
        this.hopper = hopper;
        this.indexer = indexer;
        this.pooper = pooper;
        this.intakeRoller = intakeRoller;
        addRequirements(hopper, indexer, pooper, colorSensor, intakeRoller);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
//        if (hopper.isBallPresent()) {
////            hopper.driveHopper(0.0);
////            if (!colorSensor.isTeamColor()) {
////                pooper.extend();
//            }
//        } else {
//            hopper.driveHopper(Constants.MotorSpeeds.HOPPER_SPEED);
//            if (pooper.isExtended()) {
//                pooper.retract();
//            }
//        }
        if (indexer.isBallPresent()) {
            indexer.drive(0.0);
        } else {
            indexer.drive(Constants.MotorSpeeds.INDEXER_SPEED);
        }

        if (indexer.isBallPresent() && hopper.isBallPresent()) {
            indexer.drive(0.0);
            intakeRoller.setRollerSpeed(0.0);
        } else {
            hopper.driveHopper(Constants.MotorSpeeds.HOPPER_SPEED);
            intakeRoller.setRollerSpeed(Constants.MotorSpeeds.INTAKE_SPEED);
        }

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.drive(0.0);
        intakeRoller.setRollerSpeed(0.0);
        indexer.drive(0.0);
    }
}
