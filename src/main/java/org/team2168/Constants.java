// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.utils.Gains;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
    
    public static final boolean IS_COMPBOT = new DigitalInput(DIO.PRACTICEBOT_JUMPER).get();
    public static final class Joysticks {
        public static final int DRIVER_JOYSTICK = 0;
        public static final int OPERATOR_JOYSTICK = 1;
        public static final int BUTTON_BOX_1 = 2;
        public static final int BUTTON_BOX_2 = 3;
        public static final int DRIVER_OPERATOR_E_BACKUP = 4;
        public static final int PID_TEST_JOYSTICK = 5;
    }

    public static final class CANDevices {
        public static final int DRIVETRAIN_RIGHT_MOTOR_1 = 18;
        public static final int DRIVETRAIN_RIGHT_MOTOR_2 = 19;
        public static final int DRIVETRAIN_LEFT_MOTOR_1 = 1;
        public static final int DRIVETRAIN_LEFT_MOTOR_2 = 2;
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

    public static final class Drivetrain {
        public static final double ksVolts = 0.20;
        public static final double kvVoltSecondsPerMeter = 2.3087;
        public static final double kaVoltSecondsSquaredPerMeter = 0.73973;
        public static final double kPDriveVel = 2.0; // @1.4 m/s max error; 1.78

        public static final double kTrackwidthMeters = 0.67813;//what sysid claims:0.73295/Actual = 0.65563;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3.2;  // 3.7
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.4;  // 4.0 is feasable?

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;//worked for 1m/s = 8.0;// stock 2;
        public static final double kRamseteZeta = 0.7; // worked for 1m/s 0.9; //stock 0.7 worked straight @ 0.85

        public static final double MAX_VOLTAGE = 10.0;
        // public static final double MAX_VELOCITY = 3.7;  // 4.8 works on a crispy battery; 3.7 slow but steady
        // public static final double MAX_ACCEL = 0.7;  // 1.4 works on a crispy battery; 0.7 slow but steady

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
         * Not all set of Gains are used in this project and may be removed as desired.
         *
         * 0.21 @ 12v minimum straight output
         * 	                                        	               kP     kI     kD   kF           Iz    PeakOut */
        public final static Gains kGains_Distance         = new Gains( 0.125, 0.0, 0.0, 0.0,           120,  0.75 ); //always used for linear path
        public final static Gains kGains_Turning          = new Gains( 20.0, 0.0, 0.0, 1023 * 0.08,    200,  0.5 );  //used to turn during autos
        public final static Gains kGains_Turning_Straight = new Gains( 20.0, 1.0, 0.0, 1023 * 0.17,    300,  0.50 ); //used to maintain heading while auto driving straight
        public final static Gains kGains_Limelight        = new Gains( 0.55, 0.0, 0.0, 1023.0/6800.0,  200,  0.5 );
	
        /** ---- Flat constants, you should not need to change these ---- */
        /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
        public final static int REMOTE_0 = 0;
        public final static int REMOTE_1 = 1;
        /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
        public final static int PID_PRIMARY = 0;
        public final static int PID_TURN = 1;
        /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
        public final static int SLOT_0 = 0;
        public final static int SLOT_1 = 1;
        public final static int SLOT_2 = 2;
        public final static int SLOT_3 = 3;
        /* ---- Named slots, used to clarify code ---- */
        public final static int kSlot_Distanc = SLOT_0;
        public final static int kSlot_Turning = SLOT_1;
        public final static int kSlot_Velocit = SLOT_2;
        public final static int kSlot_MotProf = SLOT_3;

        public final static int kPigeonUnitsPerRotation = 8192;

    }

    public static final class PneumaticsDevices {
        public static final int MONKEYBAR_RETRACT = 0;
        public static final int MONKEYBAR_EXTEND = 1;
        public static final int POOPER_DOUBLE_SOLENOID_EXTEND = 2;
        public static final int POOPER_DOUBLE_SOLENOID_RETRACT = 3;
        public static final int RED_LED = 5;
        public static final int GREEN_LED = 6;
        public static final int BLUE_LED = 7;
        public static final int INTAKE_RAISE = 12;
        public static final int INTAKE_LOWER = 13;
        public static final int TEST_RETRACT = 14; 
        public static final int TEST_EXTEND = 15;

        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
    }

    public static final class DIO {
        public static final int MONKEYBAR_LIMIT_SWITCH = 0;
        public static final int CLIMBER_HOOK_LIMIT_SWITCH = 1;
        public static final int HOPPER_LINE_BREAK = 2;
        public static final int INDEXER_SENSOR = 3;
        public static final int PRACTICEBOT_JUMPER = 7; //because its easier to reach than 9
    }

    public static final class MotorSpeeds {
        // Default speeds to input into commands
        public static final double INDEXER_SPEED;
        public static final double HOPPER_SPEED;
        public static final double INTAKE_SPEED;
        static {
            if(IS_COMPBOT) {
                INDEXER_SPEED = 0.3;
                HOPPER_SPEED = 0.5;
                INTAKE_SPEED = 1.0;
            } else {
                INDEXER_SPEED = 0.3;
                HOPPER_SPEED = 0.5;
                INTAKE_SPEED = 1.0;
            }
        }
    }
    
    public static final class LiftPositions {
        public static final double LIFT_ABOVE_BAR_FROM_AIR_INCHES = 29.8;
        public static final double LIFT_ABOVE_BAR_FROM_GROUND_INCHES = 27.0;
        public static final double LIFT_EXTEND_BELOW_NEXT_BAR_INCHES = 19.0;
        public static final double LIFT_RETRACT_TO_MINIMIZE_TRAVESE_SWING_INCHES = 19.0;

        public static final double LIFT_UNLOAD_TO_MBAR_INCHES = 4.5;        // raise lift to clear bar prior to mbar tilt
        public static final double LIFT_ARRESTING_INCHES = 1.8;
        public static final double LIFT_RETRACTION_INCHES = -0.2168/1.678;  // lower position to engage into the monkey bars
                                                                            // currently need to lower past zero to take up backlash and in gears when under load    
        public static final double LIFT_ZERO_INCHES = 0.0;

        public static final double SAFE_TRAVERSE_BAR_EXTEND_PITCH = 30.0;
        public static final double SAFE_HIGH_BAR_EXTEND_PITCH = 30.0;
        public static final double TOO_CLOSE_TO_SWING_APEX_PITCH = 40.0;
        public static final double TOO_CLOSE_TO_SWING_APEX_PITCH_HIGH = 90.0; // there is no minimum value here!
        public static final double LIFT_UNLOAD_TO_MBAR_PITCH = 15.0;
    }

    public static final class Heights {
        public static final double ROBOT_LIMELIGHT_HEIGHT_METERS = 1.00838; // 39.7 in.
        public static final double UPPER_HUB_HEIGHT_METERS = 2.6416; // 104 in.
    }

    public static final class Distances {
        public static final double LIMELIGHT_OFFSET_METERS = 0.76195; // 14.25 in. + extra offset to give accurate distances
    }

    public static final class Analog {
        public static final int TURRET_POTENTIOMETER = 0;
    }

    public static final double LOOP_TIMESTEP_S = 0.02;
}
