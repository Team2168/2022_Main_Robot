// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Joysticks {
        public static final int DRIVER_JOYSTICK = 0;
        public static final int OPERATOR_JOYSTICK = 1;
        public static final int BUTTON_BOX_1 = 2;
        public static final int BUTTON_BOX_2 = 3;
        public static final int DRIVER_OPERATOR_E_BACKUP = 4;
        public static final int PID_TEST_JOYSTICK = 5;
    }

    public static final class CANDevices {
        public static final int DRIVETRAIN_RIGHT_MOTOR_1 = 0;
        public static final int DRIVETRAIN_RIGHT_MOTOR_2 = 1;
        public static final int DRIVETRAIN_RIGHT_MOTOR_3 = 2;
        public static final int DRIVETRAIN_LEFT_MOTOR_1 = 15;
        public static final int DRIVETRAIN_LEFT_MOTOR_2 = 14;
        public static final int DRIVETRAIN_LEFT_MOTOR_3 = 13;
        public static final int PIGEON_IMU = 17;
    }

    public static final class Drivetrain {
        public static final double ksVolts = 0.61421;
        public static final double kvVoltSecondsPerMeter = 2.007;
        public static final double kaVoltSecondsSquaredPerMeter = 0.60831;
        public static final double kPDriveVel = 2.8231; //2.8231;

        public static final double kTrackwidthMeters = 0.87;//fudged to approximately right. Actual = 0.65;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;//worked for 1m/s = 8.0;// stock 2;
        public static final double kRamseteZeta = 0.7; // worked for 1m/s 0.9; //stock 0.7
    }
}

