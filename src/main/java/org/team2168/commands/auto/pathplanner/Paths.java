package org.team2168.commands.auto.pathplanner;

import java.io.IOException;

import org.team2168.Constants;
import org.team2168.utils.PathUtil;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class to generate trajectories before autos begins.
 * 
 * This isn't explicitly needed, but it's an optimization
 * especially for running multiple paths in quick succession
 */
public class Paths {

    public final Trajectory path_4BALL_0;
    public final Trajectory path_4BALL_1;
    public final Trajectory path_4BALL_2;
    public final Trajectory path_4BALL_3;
    public final Trajectory path_4BALL_AIO;
    public final Trajectory path_2BALL_1;
    public final Trajectory path_3BALL_0;
    public final Trajectory path_3BALL_1;
    public final Trajectory path_Drive1Meter;
    public final Trajectory path_Drive3Meters;
    public final Trajectory path_Disturb_1;
    public final Trajectory path_Disturb_2;
    public final Trajectory path_Disturb_3;
    public final Trajectory path_Disturb_4;
    public final Trajectory path_Disturb_5;
    public final Trajectory path_Disturb_6;
    public final Trajectory path_Disturb_7;

    private static Paths instance = null;

    private Paths() {
        System.out.println("******* Begin generating autos *******");


        path_4BALL_0 = getTrajectory("4BALL_0", true);
        path_4BALL_1 = getTrajectory("4BALL_1", true);
        path_4BALL_2 = getTrajectory("4BALL_2", true);
        path_4BALL_3 = getTrajectory("4BALL_3", false);
        path_4BALL_AIO = getTrajectory("4BALL_AIO", true);
        path_2BALL_1 = getTrajectory("2BALL_1", true);
        path_3BALL_0 = getTrajectory("4BALL_0", true, 3.7, 0.7);
        path_3BALL_1 = getTrajectory("3BALL_1", true, 3.7, 0.7);
        path_Drive1Meter = getTrajectory("Drive1Meter", true);
        path_Drive3Meters = getTrajectory("Drive3Meters", true);
        path_Disturb_1 = getTrajectory("Disturb_1", true);
        path_Disturb_2 = getTrajectory("Disturb_2", false);
        path_Disturb_3 = getTrajectory("Disturb_3", true);
        path_Disturb_4 = getTrajectory("Disturb_4", true);
        path_Disturb_5 = getTrajectory("Disturb_5", true);
        path_Disturb_6 = getTrajectory("Disturb_6", true);
        path_Disturb_7 = getTrajectory("Distub_7", false);

        
        System.out.println("******* Finish generating autos *******");
    }

    private Trajectory getTrajectory(String pathName, boolean reversed) {
        return this.getTrajectory(pathName, reversed, Constants.Drivetrain.kMaxSpeedMetersPerSecond, Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared);
    }
    private Trajectory getTrajectory(String pathName, boolean reversed, double vel, double accel) {
        try {
            return PathUtil.getPathPlannerTrajectory(pathName, reversed, vel, accel);
        } catch (IOException e) {
            final String ERRORMESSAGE = "Failed to read path %s!  Check the file name.  Falling back to empty trajectory.";
            DriverStation.reportError(String.format(ERRORMESSAGE, pathName), e.getStackTrace());
            return new Trajectory();
        }
    }

    public static Paths getInstance() {
        if (instance == null)
            instance = new Paths();
        return instance;
    }

}
