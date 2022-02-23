// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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

        public static final int DRIVETRAIN_RIGHT_MOTOR_1 = 17;
        public static final int DRIVETRAIN_RIGHT_MOTOR_2 = 18;
        public static final int DRIVETRAIN_RIGHT_MOTOR_3 = 19;
        public static final int DRIVETRAIN_LEFT_MOTOR_1 = 0;
        public static final int DRIVETRAIN_LEFT_MOTOR_2 = 1;
        public static final int DRIVETRAIN_LEFT_MOTOR_3 = 2;
        public static final int CLIMBER_MOTOR_RIGHT = 3;
        public static final int CLIMBER_MOTOR_LEFT = 16;
        public static final int INDEXER_MOTOR = 12;
        public static final int INTAKE_MOTOR = 15;
        public static final int HOPPER_MOTOR = 5;
        public static final int SHOOTER_LEFT_MOTOR = 7;
        public static final int SHOOTER_RIGHT_MOTOR = 8;
        public static final int HOOD_MOTOR = 9;
        public final static int TURRET_MOTOR = 13;
        public static final int PIGEON_IMU = 20;
    }

    public static final class PneumaticsDevices {
        public static final int MONKEYBAR_EXTEND = 0;
        public static final int MONKEYBAR_RETRACT = 1;
        public static final int POOPER_DOUBLE_SOLENOID_EXTEND = 2;
        public static final int POOPER_DOUBLE_SOLENOID_RETRACT = 3;
        public static final int TEST_RETRACT = 14; 
        public static final int TEST_EXTEND = 15;
        public static final int INTAKE_RAISE = 12;
        public static final int INTAKE_LOWER = 13;

        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
    }

    public static final class DIO {
        public static final int MONKEYBAR_LIMIT_SWITCH = 0;
        public static final int CLIMBER_HOOK_LIMIT_SWITCH = 1;
        public static final int HOPPER_LINE_BREAK = 2;
        public static final int INDEXER_SENSOR = 3;
    }
    public static final double LOOP_TIMESTEP_S = 0.02;
}
