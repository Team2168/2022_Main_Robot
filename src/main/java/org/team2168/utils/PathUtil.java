package org.team2168.utils;

import java.io.IOException;

import com.pathplanner.lib.PathPlanner;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathUtil {

    private PathUtil() {
        throw new UnsupportedOperationException("Do not instantiate utility classes!");
    }

    public static enum InitialPathState {
        PRESERVEHEADING,
        PRESERVEODOMETRY,
        DISCARDHEADING,
    }

    public static TrajectoryConstraint getVoltageConstraint() {
        return new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.Drivetrain.ksVolts,
                        Constants.Drivetrain.kvVoltSecondsPerMeter,
                        Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
                Constants.Drivetrain.kDriveKinematics,
                10);
    }

    public static RamseteCommand getRamseteCommand(Trajectory trajectory, Drivetrain drivetrain) {
        return new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(Constants.Drivetrain.kRamseteB, Constants.Drivetrain.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.Drivetrain.ksVolts,
                        Constants.Drivetrain.kvVoltSecondsPerMeter,
                        Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
                Constants.Drivetrain.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
                new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
    }

    public static Command getPathCommand(Trajectory trajectory, Drivetrain drivetrain, InitialPathState initialState) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        try {
            var initialPose = trajectory.getInitialPose();

            switch (initialState) {
                case PRESERVEODOMETRY:
                    break;
                case PRESERVEHEADING:
                    sequence.addCommands(
                            new InstantCommand(() -> drivetrain.resetOdometry(initialPose, true)));
                    break;
                case DISCARDHEADING:
                    sequence.addCommands(
                            // new ResetHeading(drivetrain), // TODO reimpliment this once stuff works
                            new InstantCommand(() -> drivetrain.resetOdometry(initialPose, true)));
                    break;
            }
        } catch (IndexOutOfBoundsException e) {
            DriverStation.reportError("Cannot parse an empty path!  Falling back to no path.", e.getStackTrace());
            return new InstantCommand();
        }
        sequence.addCommands(
                new InstantCommand(() -> System.out.println("GYRO HEADING @ PATH START: " + drivetrain.getHeading())));
        sequence.addCommands(getRamseteCommand(trajectory, drivetrain).andThen(() -> drivetrain.tankDrive(0.0, 0.0)));
        return sequence;
    }

    public static Command getPathPlannerCommand(String pathName, Drivetrain drivetrain, InitialPathState initialState,
            boolean reverse) {
        try {
            var path = getPathPlannerTrajectory(pathName, reverse);
            return getPathCommand(path, drivetrain, initialState);
        } catch (IOException e) {
            final String ERRORMESSAGE = "Failed to read path %s!  Doing nothing instead of crashing.";
            DriverStation.reportError(String.format(ERRORMESSAGE, pathName), e.getStackTrace());
            return new InstantCommand();

        }
    }

    public static Command getPathPlannerCommand(String pathName, Drivetrain drivetrain, InitialPathState initialState) {
        return PathUtil.getPathPlannerCommand(pathName, drivetrain, initialState, true); // we will usually be driving backwards
    }

    public static Command getPathPlannerCommand(String pathName, Drivetrain drivetrain) {
        return PathUtil.getPathPlannerCommand(pathName, drivetrain, InitialPathState.DISCARDHEADING, true);
    }

    public static Trajectory getPathPlannerTrajectory(String pathName, boolean reverse, double maxVel, double maxAccel) throws IOException {
        var path = PathPlanner.loadPath(pathName, maxVel, maxAccel, reverse);
        if (path == null) {
            // PathPlanner doesn't throw an error if it can't load a path; instead it fails
            // silently and returns null which is bad
            final String error = "PathPlanner could not parse path %s!";
            throw new IOException(String.format(error, pathName));
        }
        return path;
    }
    public static Trajectory getPathPlannerTrajectory(String pathName, boolean reverse) throws IOException {
        return PathUtil.getPathPlannerTrajectory(pathName, reverse, Constants.Drivetrain.kMaxSpeedMetersPerSecond, Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared);
    }

    /**
     * Gets path command from trajectory file name.
     * 
     * @see #getTrajectory getTrajectory for more information on the expected path
     *      name.
     * @param pathName     name of path json
     * @param drivetrain   Drivetrain object
     * @param initialState
     * @return path command
     */
    public static Command getPathWeaverCommand(String pathName, Drivetrain drivetrain, InitialPathState initialState) {
        try {
            return PathUtil.getPathCommand(getPathWeaverTrajectory(pathName), drivetrain, initialState);
        } catch (IOException e) {
            final String ERRORMESSAGE = "Failed to read path %s!  Doing nothing instead of crashing.";
            DriverStation.reportError(String.format(ERRORMESSAGE, pathName), e.getStackTrace());
            return new InstantCommand();
        }
    }

    public static Command getPathWeaverCommand(String pathName, Drivetrain drivetrain) {
        return PathUtil.getPathWeaverCommand(pathName, drivetrain, InitialPathState.DISCARDHEADING);
    }

    /**
     * Gets a trajectory which is stored in deploy/paths.
     * 
     * <b>If a path cannot be read, this command returns an empty path instead of
     * crashing!</b>
     * It will also output to DS logs.
     * 
     * The name of the path file does not include .wpilib.json!
     * 
     * @param pathName The name of the path file
     * @throws IOException if path file does not exist
     * @return Computed trajectory
     */
    public static Trajectory getPathWeaverTrajectory(String pathName) throws IOException {
        final String PATHSDIR = "paths/%s.wpilib.json"; // directory of path jsons from within deploy directory
        var trajectoryFSPath = Filesystem.getDeployDirectory().toPath().resolve(String.format(PATHSDIR, pathName));

        return TrajectoryUtil.fromPathweaverJson(trajectoryFSPath);
    }

}
