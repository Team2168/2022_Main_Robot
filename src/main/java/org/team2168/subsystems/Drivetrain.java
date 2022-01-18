// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import org.team2168.Constants.CANDevices;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX leftMotor1;
    private WPI_TalonFX leftMotor2;
    private WPI_TalonFX leftMotor3;
    private WPI_TalonFX rightMotor1;
    private WPI_TalonFX rightMotor2;
    private WPI_TalonFX rightMotor3;
    private WPI_PigeonIMU pidgey; // Same as normal pigeon; implements wpi methods

    private DifferentialDrive drive;
    private DifferentialDriveOdometry odometry;

    private static Drivetrain instance = null;

    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT = 40; // amps
    private static final double TRIGGER_THRESHOLD_LIMIT = 60; // amp
    private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s
    private final static double NEUTRALDEADBAND = 0.001;

    private SupplyCurrentLimitConfiguration talonCurrentLimit;

    public static final boolean DT_REVERSE_LEFT1 = false;
    public static final boolean DT_REVERSE_LEFT2 = false;
    public static final boolean DT_REVERSE_LEFT3 = false;
    public static final boolean DT_REVERSE_RIGHT1 = true;
    public static final boolean DT_REVERSE_RIGHT2 = true;
    public static final boolean DT_REVERSE_RIGHT3 = true;
    public static final boolean DT_3_MOTORS_PER_SIDE = true;

    /** Invert Directions for Left and Right */
    TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; // Same as invert = "false"
    TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; // Same as invert = "true"

    /** Config Objects for motor controllers */
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    public static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
    public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
    public static final double GEAR_RATIO = (50.0 / 10.0) * (40.0 / 22.0);
    public static final double WHEEL_DIAMETER = 4.0; // inches
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // inches
    public static final double PIGEON_UNITS_PER_ROTATION = 8192.0;
    public static final double DEGREES_PER_REV = 360.0;
    public static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
    public static final double WHEEL_BASE = 24.0; // distance between wheels (width) in inches

    /**
     * Gets the singleton instance of the drivetrain
     * 
     * @return drivetrain
     */
    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    /**
     * Creates a new Drivetrain.
     * This Drivetrain is configured to have 2 front "leader" motors, and 4 rear
     * "follower" motors
     * 
     * Each motor in the drivetrain is coupled together mechanically, so there isn't
     * much of a point in
     * giving each motor freedom.
     */
    private Drivetrain() {
        // Instantiate motor objects
        leftMotor1 = new WPI_TalonFX(CANDevices.DRIVETRAIN_LEFT_MOTOR_1);
        leftMotor2 = new WPI_TalonFX(CANDevices.DRIVETRAIN_LEFT_MOTOR_2);
        leftMotor3 = new WPI_TalonFX(CANDevices.DRIVETRAIN_LEFT_MOTOR_3);
        rightMotor1 = new WPI_TalonFX(CANDevices.DRIVETRAIN_RIGHT_MOTOR_1);
        rightMotor2 = new WPI_TalonFX(CANDevices.DRIVETRAIN_RIGHT_MOTOR_2);
        rightMotor3 = new WPI_TalonFX(CANDevices.DRIVETRAIN_RIGHT_MOTOR_3);

        // Instantiate gyro
        pidgey = new WPI_PigeonIMU(CANDevices.PIGEON_IMU);

        // Reset the configurations on the motor controllers
        leftMotor1.configFactoryDefault();
        leftMotor2.configFactoryDefault();
        leftMotor3.configFactoryDefault();
        rightMotor1.configFactoryDefault();
        rightMotor2.configFactoryDefault();
        rightMotor3.configFactoryDefault();

        // Create a current limit
        talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
                CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

        // Add the current limit to the motor configuration object
        leftConfig.supplyCurrLimit = talonCurrentLimit;
        rightConfig.supplyCurrLimit = talonCurrentLimit;

        // Configure drivetrain to use integrated sensors
        leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        leftConfig.neutralDeadband = NEUTRALDEADBAND;
        rightConfig.neutralDeadband = NEUTRALDEADBAND;

        leftMotor1.configAllSettings(leftConfig);
        leftMotor2.configAllSettings(leftConfig);
        leftMotor3.configAllSettings(leftConfig);
        rightMotor1.configAllSettings(rightConfig);
        rightMotor2.configAllSettings(rightConfig);
        rightMotor3.configAllSettings(rightConfig);

        leftMotor2.follow(leftMotor1);
        leftMotor3.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
        rightMotor3.follow(rightMotor1);

        leftMotor1.setInverted(leftInvert);
        leftMotor2.setInverted(InvertType.FollowMaster);
        leftMotor3.setInverted(InvertType.FollowMaster);
        rightMotor1.setInverted(rightInvert);
        rightMotor2.setInverted(InvertType.FollowMaster);
        rightMotor3.setInverted(InvertType.FollowMaster);

        setMotorsBrake();
        drive = new DifferentialDrive(leftMotor1, rightMotor1);
        odometry = new DifferentialDriveOdometry(pidgey.getRotation2d());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(pidgey.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    /**
     * Gets the odometry pose
     * 
     * @return Pose2d odometry pose
     */
    @Log(name = "Robot Pose", rowIndex = 3, columnIndex = 0)
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Gets gyro heading
     * 
     * @return gyro heading from -180.0 to 180.0 degrees
     */
    @Log(name = "Gyro Heading", rowIndex = 2, columnIndex = 1)
    public double getHeading() {
        return pidgey.getRotation2d().getDegrees();
    }

    /**
     * Gets gyro turn rate
     * 
     * @return rate in degrees per second
     */
    @Log(name = "Turn velocity", rowIndex = 2, columnIndex = 0)
    public double getTurnRate() {
        return -pidgey.getRate();
    }

    /**
     * Get average encoder distance
     * 
     * @return gets distance in meters
     */
    @Log(name = "Average Encoder Distance (m)", rowIndex = 1, columnIndex = 0)
    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }

    /**
     * Gets wheel speeds in meters per second
     * 
     * @return DifferentialDriveWheelSpeeds object
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
    }

    /**
     * Gets left encoder distance
     * 
     * @return encoder distance in meters
     */
    @Log(name = "Left Encoder Distance (m)", rowIndex = 1, columnIndex = 1)
    public double getLeftEncoderDistance() {
        return ticksToMeters(leftMotor1.getSelectedSensorPosition());
    }

    /**
     * Gets right encoder distance
     * 
     * @return encoder distance in meters
     */
    @Log(name = "Right Encoder Distance (m)", rowIndex = 1, columnIndex = 2)
    public double getRightEncoderDistance() {
        return ticksToMeters(rightMotor1.getSelectedSensorPosition());
    }

    /**
     * Zeroes gyro heading
     */
    public void zeroHeading() {
        pidgey.reset();
    }

    /**
     * Resets encoders on motors
     */
    public void resetEncoders() {
        leftMotor1.setSelectedSensorPosition(0.0);
        rightMotor1.setSelectedSensorPosition(0.0);
    }

    /**
     * Resets odometry to specified pose
     * 
     * @param pose            pose to set odometry
     * @param preserveHeading do we preserve the gyro heading?
     */
    public void resetOdometry(Pose2d pose, boolean preserveHeading) {
        resetEncoders();
        if (!preserveHeading)
            zeroHeading();
        odometry.resetPosition(pose, pidgey.getRotation2d());
    }

    /**
     * Reset odometry to specified pose, while resetting the gyro.
     * 
     * @param pose pose to set odometry
     */
    public void resetOdometry(Pose2d pose) {
        this.resetOdometry(pose, false);
    }

    /**
     * Gets Left encoder velocity
     * @return velocity in sensor ticks
     */
    public double getLeftEncoderRateRaw() {
        return leftMotor1.getSelectedSensorVelocity();
    }

    /**
     * Gets Right encoder velocity
     * @return velocity in sensor ticks
     */
    public double getRightEncoderRateRaw() {
        return rightMotor1.getSelectedSensorVelocity();
    }
    /**
     * Gets left encoder velocity
     * 
     * @return encoder velocity in meters/second
     */
    @Log(name = "Left Velocity", rowIndex = 4, columnIndex = 0)
    public double getLeftEncoderRate() {
        return ticksToMeters(getLeftEncoderRateRaw()) * 10.0;
    }

    /**
     * Gets left encoder velocity
     * 
     * @return encoder velocity in meters/second
     */
    @Log(name = "Right Velocity", rowIndex = 4, columnIndex = 1)

    public double getRightEncoderRate() {
        return ticksToMeters(getRightEncoderRateRaw()) * 10.0;
    }

    private double ticksToInches(double setpoint) {
        return (setpoint * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * GEAR_RATIO);
    }

    private double ticksToMeters(double ticks) {
        // the cheezy poofs have been colluding to get inside of our codebase
        return ticksToInches(ticks) * 0.0254;
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * Change all motors to their default mix of brake/coast modes.
     * Should be used for normal match play.
     */
    public void setMotorsBrake() {
        leftMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.setNeutralMode(NeutralMode.Coast);
        leftMotor3.setNeutralMode(NeutralMode.Coast);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor2.setNeutralMode(NeutralMode.Coast);
        rightMotor3.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Change all the drivetrain motor controllers to coast mode.
     * Useful for allowing robot to be manually pushed around the field.
     */
    public void setMotorsCoast() {
        leftMotor1.setNeutralMode(NeutralMode.Coast);
        leftMotor2.setNeutralMode(NeutralMode.Coast);
        leftMotor3.setNeutralMode(NeutralMode.Coast);
        rightMotor1.setNeutralMode(NeutralMode.Coast);
        rightMotor2.setNeutralMode(NeutralMode.Coast);
        rightMotor3.setNeutralMode(NeutralMode.Coast);
    }

}
