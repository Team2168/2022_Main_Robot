package org.team2168.commands.auto;

import java.io.IOException;

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
    public final Trajectory path_2BALL_1;
    public final Trajectory path_canweturn;
    public final Trajectory path_wecanturn;
    public final Trajectory path_Drive1Meter;
    public final Trajectory path_Drive3Meters;

    private static Paths instance = null;

    private Paths() {
        System.out.println("******* Begin generating autos *******");


        path_4BALL_0 = getTrajectory("4BALL_0", true);
        path_4BALL_1 = getTrajectory("4BALL_1", true);
        path_4BALL_2 = getTrajectory("4BALL_2", true);
        path_4BALL_3 = getTrajectory("4BALL_3", true);
        path_2BALL_1 = getTrajectory("2BALL_1", true);
        path_canweturn = getTrajectory("canweturn", true);
        path_wecanturn = getTrajectory("wecanturn", true);
        path_Drive1Meter = getTrajectory("Drive1Meter", true);
        path_Drive3Meters = getTrajectory("Drive3Meters", true);

        
        System.out.println("******* Finish generating autos *******");
    }

    private Trajectory getTrajectory(String pathName, boolean reversed) {
        try {
            return PathUtil.getPathPlannerTrajectory(pathName, reversed);
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
