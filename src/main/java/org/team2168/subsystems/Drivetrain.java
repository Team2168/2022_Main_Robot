// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.kauailabs.navx.frc.AHRS;

import org.team2168.Constants;
import org.team2168.Constants.CANDevices;
import org.team2168.utils.PigeonHelper;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
    private PigeonHelper pidgey; // Same as normal pigeon; implements wpi methods

    private TalonFXHelper leftMotor1;
    private TalonFXHelper leftMotor2;
    private TalonFXHelper rightMotor1;
    private TalonFXHelper rightMotor2;

    // private DifferentialDrive drive;
    private DifferentialDriveOdometry odometry;

    private static Drivetrain instance = null;

    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT = 35; // amps
    private static final double TRIGGER_THRESHOLD_LIMIT = 40; // amp
    private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s
    private final static double NEUTRALDEADBAND = 0.001;

    private SupplyCurrentLimitConfiguration talonCurrentLimit;

    /**
     * Invert Directions for Left and Right
     */
    TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; // Same as invert = "false"
    TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; // Same as invert = "true"

    /**
     * Config Objects for motor controllers
     */
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    public static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
    public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
    public static final double GEAR_RATIO = (50.0/10.0) * (38.0/28.0);
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // inches
    public static final double PIGEON_UNITS_PER_ROTATION = 8192.0;
    public static final double DEGREES_PER_REV = 360.0;
    public static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
    public static final double WHEEL_BASE = 24.0; // distance between wheels (width) in inches
    public static final int TIMEOUT = 30;  // 30ms

    private double setPointPosition_sensorUnits;
    private double setPointHeading_sensorUnits;

    private TalonFXConfiguration straightConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turnConfig = new TalonFXConfiguration();

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
     * <p>
     * Each motor in the drivetrain is coupled together mechanically, so there isn't
     * much of a point in
     * giving each motor freedom.
     */
    private Drivetrain() {
        // Instantiate motor objects
        leftMotor1 = new TalonFXHelper(CANDevices.DRIVETRAIN_LEFT_MOTOR_1);
        leftMotor2 = new TalonFXHelper(CANDevices.DRIVETRAIN_LEFT_MOTOR_2);
        rightMotor1 = new TalonFXHelper(CANDevices.DRIVETRAIN_RIGHT_MOTOR_1);
        rightMotor2 = new TalonFXHelper(CANDevices.DRIVETRAIN_RIGHT_MOTOR_2);

        // Instantiate gyro
        pidgey = new PigeonHelper(CANDevices.PIGEON_IMU);


        leftMotor1.configFactoryDefault();
        leftMotor2.configFactoryDefault();
        rightMotor1.configFactoryDefault();
        rightMotor2.configFactoryDefault();

        leftMotor1.configFollowerStatusFrameRates();
        leftMotor2.configFollowerStatusFrameRates();
        rightMotor1.configClosedLoopStatusFrameRates();
        rightMotor2.configFollowerStatusFrameRates();

        // Create a current limit
        talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
                CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

        // Add the current limit to the motor configuration object
        leftConfig.supplyCurrLimit = talonCurrentLimit;
        rightConfig.supplyCurrLimit = talonCurrentLimit;

        // Configure drivetrain to use integrated sensors
        leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        // leftConfig.neutralDeadband = NEUTRALDEADBAND;
        // rightConfig.neutralDeadband = NEUTRALDEADBAND;

        // pid for distance
        rightConfig.slot0.kF = Constants.Drivetrain.kGains_Distance.kF;
        rightConfig.slot0.kP = Constants.Drivetrain.kGains_Distance.kP;
        rightConfig.slot0.kI = Constants.Drivetrain.kGains_Distance.kI;
        rightConfig.slot0.kD = Constants.Drivetrain.kGains_Distance.kD;
        rightConfig.slot0.integralZone = Constants.Drivetrain.kGains_Distance.kIzone;
        rightConfig.slot0.closedLoopPeakOutput = Constants.Drivetrain.kGains_Distance.kPeakOutput;

        // heading pid
        rightConfig.remoteFilter1.remoteSensorDeviceID = pidgey.getDeviceID();    //Pigeon Device ID
        rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
        rightConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1; //Set as the Aux Sensor
        rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.Drivetrain.kPigeonUnitsPerRotation; //Convert Yaw to tenths of a degree

        /* false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
         *   This is typical when the master is the right Talon FX and using Pigeon
         *
         * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
         *   This is typical when the master is the left Talon FX and using Pigeon
         */
        rightConfig.auxPIDPolarity = true;

        /**
         * 1ms per loop.  PID loop can be slowed down if need be.
         * For example,
         * - if sensor updates are too slow
         * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
         * - sensor movement is very slow causing the derivative error to be near zero.
         */
        int closedLoopTimeMs = 1;
        rightMotor1.configClosedLoopPeriod(0, closedLoopTimeMs, TIMEOUT);
        rightMotor1.configClosedLoopPeriod(1, closedLoopTimeMs, TIMEOUT);



        leftMotor1.configAllSettings(leftConfig);
//        leftMotor2.configAllSettings(leftConfig);
        rightMotor1.configAllSettings(rightConfig);
//        rightMotor2.configAllSettings(rightConfig);

        rightMotor1.selectProfileSlot(Constants.Drivetrain.kSlot_Distanc, Constants.Drivetrain.PID_PRIMARY);
        rightMotor1.selectProfileSlot(Constants.Drivetrain.kSlot_Turning, Constants.Drivetrain.PID_TURN);

        leftMotor1.configVoltageCompSaturation(Constants.Drivetrain.MAX_VOLTAGE);
        leftMotor1.enableVoltageCompensation(true);
        rightMotor1.configVoltageCompSaturation(Constants.Drivetrain.MAX_VOLTAGE);
        rightMotor1.enableVoltageCompensation(true);

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        leftMotor1.setInverted(leftInvert);
        leftMotor2.setInverted(InvertType.FollowMaster);
        rightMotor1.setInverted(rightInvert);
        rightMotor2.setInverted(InvertType.FollowMaster);

        setMotorsBrake();
        // drive = new DifferentialDrive(leftMotor1, rightMotor1);
        // drive.setDeadband(0.0);  // Disable differentialDrive deadband; deadband is handled by the controllers

        pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 1);  // status frame in ms
        odometry = new DifferentialDriveOdometry(pidgey.getRotation2d());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(pidgey.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    public void switchGains(boolean straightmode) {
        if(straightmode) {
            /* Motion Magic Configs */
            rightMotor1.configMotionAcceleration((int) (inchesPerSecToTicksPer100ms(8.0*12.0))); //(distance units per 100 ms) per second
            rightMotor1.configMotionCruiseVelocity((int) (inchesPerSecToTicksPer100ms(10.0*12.0))); //distance units per 100 ms
            /* FPID for Heading */
            rightMotor1.config_kF(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning_Straight.kF, TIMEOUT);
            rightMotor1.config_kP(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning_Straight.kP, TIMEOUT);
            rightMotor1.config_kI(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning_Straight.kI, TIMEOUT);
            rightMotor1.config_kD(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning_Straight.kD, TIMEOUT);
            rightMotor1.config_IntegralZone(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning_Straight.kIzone, TIMEOUT);
            rightMotor1.configClosedLoopPeakOutput(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning_Straight.kPeakOutput, TIMEOUT);

            rightMotor1.configNominalOutputForward(0.045, TIMEOUT);
            rightMotor1.configNominalOutputReverse(-0.045, TIMEOUT);
            rightMotor1.configPeakOutputForward(1.0, TIMEOUT);
            rightMotor1.configPeakOutputReverse(-1.0, TIMEOUT);
            leftMotor1.configNominalOutputForward(0.045, TIMEOUT);
            leftMotor1.configNominalOutputReverse(-0.045, TIMEOUT);
            leftMotor1.configPeakOutputForward(1.0, TIMEOUT);
            leftMotor1.configPeakOutputReverse(-1.0, TIMEOUT);
        } else {
            /* Motion Magic Configs */
            rightMotor1.configMotionAcceleration((int) (inchesPerSecToTicksPer100ms(8.0*12.0))); //(distance units per 100 ms) per second
            rightMotor1.configMotionCruiseVelocity((int) (inchesPerSecToTicksPer100ms(5.0*12.0))); //distance units per 100 ms
            rightMotor1.config_kF(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning.kF,TIMEOUT);
            rightMotor1.config_kP(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning.kP,TIMEOUT);
            rightMotor1.config_kI(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning.kI,TIMEOUT);
            rightMotor1.config_kD(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning.kD,TIMEOUT);
            rightMotor1.config_IntegralZone(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning.kIzone, TIMEOUT);
            rightMotor1.configClosedLoopPeakOutput(Constants.Drivetrain.SLOT_1, Constants.Drivetrain.kGains_Turning.kPeakOutput, TIMEOUT);

            rightMotor1.configNominalOutputForward(0.13, TIMEOUT);
            rightMotor1.configNominalOutputReverse(-0.13, TIMEOUT);
            rightMotor1.configPeakOutputForward(1.0, TIMEOUT);
            rightMotor1.configPeakOutputReverse(-1.0, TIMEOUT);
            leftMotor1.configNominalOutputForward(0.13, TIMEOUT);
            leftMotor1.configNominalOutputReverse(-0.13, TIMEOUT);
            leftMotor1.configPeakOutputForward(1.0, TIMEOUT);
            leftMotor1.configPeakOutputReverse(-1.0, TIMEOUT);
        }
    }


   public void teleopconfigs() {
       rightMotor1.configAllSettings(rightConfig);
       rightMotor1.configAllSettings(rightConfig);
       leftMotor1.configAllSettings(leftConfig);
       leftMotor2.configAllSettings(leftConfig);
   }

    /**
     * Gets the odometry pose
     *
     * @return Pose2d odometry pose
     */
    // @Log(name = "Robot Pose", rowIndex = 3, columnIndex = 0) //Pose2d is not a supported loggable type
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @param vel set the cruise velocity (in/sec)
     */
    public void setCruiseVelocity(double vel) {
        rightMotor1.configMotionCruiseVelocity((int) (inchesPerSecToTicksPer100ms(vel))); //distance units per 100 ms
    }

    /**
     * Gets gyro heading
     *
     * @return gyro heading from -180.0 to 180.0 degrees. Positive counterclockwise
     */
    @Log(name = "Gyro Heading", rowIndex = 2, columnIndex = 1)
    public double getHeading() {
        return pidgey.getRotation2d().getDegrees();
    }

    /**
     * Gets gyro pitch
     *
     * @return gyro pitch degrees
     */
    @Log(name = "Gyro Pitch", rowIndex = 2, columnIndex = 2)
    public double getPitch() {
        return pidgey.getPitch();
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
     * Gets left encoder distance in raw sensor units
     *
     * @return distance in sensor ticks
     */
    public double getLeftEncoderDistanceRaw() {
        return leftMotor1.getSelectedSensorPosition();
    }

    /**
     * Gets left encoder distance
     *
     * @return encoder distance in meters
     */
    @Log(name = "Left Encoder Distance (m)", rowIndex = 1, columnIndex = 1)
    public double getLeftEncoderDistance() {
        return ticksToMeters(getLeftEncoderDistanceRaw());
    }

    /**
     * Gets right encoder distance in raw sensor ticks
     *
     * @return distance in sensor ticks
     */
    public double getRightEncoderDistanceRaw() {
        return rightMotor1.getSelectedSensorPosition();
    }

    /**
     * Gets right encoder distance
     *
     * @return encoder distance in meters
     */
    @Log(name = "Right Encoder Distance (m)", rowIndex = 1, columnIndex = 2)
    public double getRightEncoderDistance() {
        return ticksToMeters(getRightEncoderDistanceRaw());
    }

    /**
     * 
     * @param setPoint distance in inches (fwd positive)
     * @param setAngle degrees
     */
    public void setSetPointPosition(double setPoint, double setAngle) {
        setPointPosition_sensorUnits = inchesToTicks(setPoint);
        setPointHeading_sensorUnits = degreesToTicks(setAngle);

        rightMotor1.set(ControlMode.MotionMagic, setPointPosition_sensorUnits, DemandType.AuxPID, setPointHeading_sensorUnits);
        rightMotor2.follow(rightMotor1, FollowerType.PercentOutput);
        leftMotor1.follow(rightMotor1, FollowerType.AuxOutput1);
        leftMotor2.follow(leftMotor1, FollowerType.PercentOutput);
    }

    public void setSetPointHeadingTeleop(double speed, double setAngle) {
        setPointHeading_sensorUnits = degreesToTicks(setAngle);

        rightMotor1.set(ControlMode.PercentOutput, speed, DemandType.AuxPID, setPointHeading_sensorUnits);
        rightMotor2.follow(rightMotor1, FollowerType.PercentOutput);
        leftMotor1.follow(rightMotor1, FollowerType.AuxOutput1);
        leftMotor2.follow(rightMotor1, FollowerType.AuxOutput1);

    }

    public double[] getMotorOutputs() {
        return new double[]{
                rightMotor1.get(), rightMotor2.get(), leftMotor1.get(), leftMotor2.get()
        };
    }

    // public void feed() {
    //     rightMotor1.feed();
    //     rightMotor2.feed();
    //     leftMotor1.feed();
    //     rightMotor1.feed();
    // }

    /**
     * 
     * @return error in inches
     */
    public double getErrorPosition() {
        return ticksToInches(setPointPosition_sensorUnits - rightMotor1.getSelectedSensorPosition(Constants.Drivetrain.PID_PRIMARY));
        //return leftMotor1.getClosedLoopError(kPIDLoopIdx)/TICKS_PER_REV;--only for nonMotionMagic or nonMotion Profile
    }

    public double getErrorHeading() {
        return ticksToDegrees(setPointHeading_sensorUnits - rightMotor1.getSelectedSensorPosition(Constants.Drivetrain.PID_TURN));
    }

    /**
     * Zeroes gyro heading
     */
    public void zeroHeading() {
        pidgey.reset();
        pidgey.setYaw(0, TIMEOUT);
        pidgey.setAccumZAngle(0, TIMEOUT);
    }

    /**
     * Resets encoders on motors
     */
    public void resetEncoders() {
        leftMotor1.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT);
		rightMotor1.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT);

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
        Rotation2d rot = pidgey.getRotation2d();

        resetEncoders();
        if (!preserveHeading)
            zeroHeading();
        odometry.resetPosition(pose, rot);
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
     *
     * @return velocity in sensor ticks
     */
    public double getLeftEncoderRateRaw() {
        return leftMotor1.getSelectedSensorVelocity();
    }

    /**
     * Gets Right encoder velocity
     *
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

    private double inchesToTicks(double setpoint) {
        return (setpoint * TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    }

    private double inchesPerSecToTicksPer100ms(double setpoint) {
        return inchesToTicks(setpoint) / 10.0;
    }

    private double ticksToInches(double setpoint) {
        return (setpoint * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * GEAR_RATIO);
    }

    private double ticksToMeters(double ticks) {
        // the cheezy poofs have been colluding to get inside of our codebase
        return ticksToInches(ticks) * 0.0254;
    }

    /**
     * Converts a setpoint in degrees to IMU 'encoder ticks'
     *
     * @param setpoint
     * @return
     */
    private double degreesToTicks(double setpoint) {
        // return (setpoint / DEGREES_PER_REV) * PIGEON_UNITS_PER_ROTATION / 2.0;
        return setpoint * 10.0;
    }

    private double ticksToDegrees(double setpoint) {
        // return (setpoint / PIGEON_UNITS_PER_ROTATION) * DEGREES_PER_REV * 2.0; 
        return setpoint / 10.0;
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        // drive.tankDrive(leftSpeed, rightSpeed);
        leftMotor1.set(leftSpeed);
        leftMotor2.set(leftSpeed);
        rightMotor1.set(rightSpeed);
        rightMotor2.set(rightSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        //TODO: Change to use voltage compensation built into the motor controllers?
        // double batteryVoltage = RobotController.getBatteryVoltage();
        tankDrive(leftVolts / Constants.Drivetrain.MAX_VOLTAGE, rightVolts / Constants.Drivetrain.MAX_VOLTAGE);

    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        tankDrive(xSpeed + zRotation, xSpeed - zRotation);
    }

    /**
     * Change all motors to their default mix of brake/coast modes.
     * Should be used for normal match play.
     */
    public void setMotorsBrake() {
        leftMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.setNeutralMode(NeutralMode.Coast);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor2.setNeutralMode(NeutralMode.Coast);
    }

    public void setMotorsBrakeAutos() {
        leftMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.setNeutralMode(NeutralMode.Brake);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor2.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Change all the drivetrain motor controllers to coast mode.
     * Useful for allowing robot to be manually pushed around the field.
     */
    public void setMotorsCoast() {
        leftMotor1.setNeutralMode(NeutralMode.Coast);
        leftMotor2.setNeutralMode(NeutralMode.Coast);
        rightMotor1.setNeutralMode(NeutralMode.Coast);
        rightMotor2.setNeutralMode(NeutralMode.Coast);
    }

    /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	 void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive total magnitude?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/

			masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	 }

}
