// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import org.team2168.Constants;
import org.team2168.Constants.CANDevices;
import org.team2168.Constants.DIO;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.Loggable;

public class Hopper extends SubsystemBase implements Loggable {

  /**
   * the hopper motor it moves the hopper
   */
  private static TalonFXHelper hopperMotor;
  
  /**
   * the hopperLineBreak, its lets the driver know the ball is entering the hopper
   */
  private DigitalInput hopperLineBreak;
  

  private static Hopper instance = null;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;

  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static final double CONTINUOUS_CURRENT_LIMIT = 40; // amps
  private static final double TRIGGER_THRESHOLD_LIMIT = 60; // amp
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s

  public static final double TICKS_PER_REV = 2048.0; 
  public static final double ONE_HUNDRED_MS_PER_MINUTE = 100.0 / 60.0;
  public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;

  public static final double GEAR_RATIO = 6.0;
  public static final double ROLLER_DIAMETER_INCHES = 1.0;
  private static final double TIME_UNITS_OF_VELOCITY = 0.1;
  public static final double ROLLER_CIRCUMFERENCE_INCHES = ROLLER_DIAMETER_INCHES * Math.PI;
  public static final double DISTANCE_PER_TICK = ROLLER_CIRCUMFERENCE_INCHES / TICKS_PER_REV;
  public static final double INCHES_PER_REV = ROLLER_DIAMETER_INCHES / 2 * Math.PI;
  public static final double HOPPER_DISTANCE_GEAR_RATIO = GEAR_RATIO * DISTANCE_PER_TICK;
  
  //Gains

  public static TalonFXInvertType hopperMotorInvert = TalonFXInvertType.CounterClockwise;
  public static final double KV = 0.02;
  public static final double KA = 0.002;

  //Simulation stuff
  
  /**
   * the hopper motor sim it simulates the hopper motor
   */
  private static TalonFXSimCollection hopperMotorSim;

  /**
   * the actual hopper sim it simulates the hopper itself
   */
  private static FlywheelSim hopperSim;
 
  public static Hopper getInstance() {
    if (instance == null)
      instance = new Hopper();
    return instance; 
  }

  /** Creates a new Hopper. */
  private Hopper() {
    hopperMotor = new TalonFXHelper(CANDevices.HOPPER_MOTOR);
    hopperLineBreak = new DigitalInput(DIO.HOPPER_LINE_BREAK);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    
    hopperMotor.configFactoryDefault();
    hopperMotor.configSupplyCurrentLimit(talonCurrentLimit);
    
    hopperMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    hopperMotor.setNeutralMode(NeutralMode.Brake);
    hopperMotor.configNeutralDeadband(0.01);

    hopperMotor.setInverted(hopperMotorInvert);

    hopperMotor.configOpenLoopStatusFrameRates();

    hopperSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(KV, KA),
      DCMotor.getFalcon500(1),
      GEAR_RATIO
    );

    hopperMotorSim = hopperMotor.getSimCollection();
  }

  /**
   * 
   * @param ticks
   * @return inches
   */

  private static double ticksToInches(double ticks) {
    return ((ticks / TICKS_PER_REV) * GEAR_RATIO * INCHES_PER_REV);
  }

  /**
   * 
   * @param inches
   * @return ticks
   */

  private static double inchesToTicks(double inches) {
    return ((inches / INCHES_PER_REV) / GEAR_RATIO * TICKS_PER_REV);
  }

  /**
   * 
   * @param speed
   */
  public void driveHopper(double speed) {
    hopperMotor.set(ControlMode.PercentOutput, speed);
  }

  

  /**
   * 
   * @return If ball enters hopper
   */
  @Log(name = "Ball Is Entering Hopper", rowIndex = 1, columnIndex = 1)
    public boolean isBallPresent() {
      return !hopperLineBreak.get();
  }

  /**
   * 
   * @return Encoder position
   */
  // @Log(name = "Encoder Position", rowIndex = 1, columnIndex = 2)
  public double getEncoderPosition() {
      return hopperMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {

    hopperMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    
    hopperSim.setInput(hopperMotorSim.getMotorOutputLeadVoltage());
    hopperSim.update(Constants.LOOP_TIMESTEP_S);

    // double sim_velocity_ticks_per_100ms = hopperSim.getAngularVelocityRPM() * ONE_HUNDRED_MS_PER_MINUTE;
    // hopperMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100ms);
    // hopperMotorSim.setIntegratedSensorRawPosition((int) (getEncoderPosition() + Constants.LOOP_TIMESTEP_S * sim_velocity_ticks_per_100ms));

    double sim_velocity_ticks_per_100_ms = inchesToTicks(Units.metersToInches(hopperSim.getAngularVelocityRPM())) * TIME_UNITS_OF_VELOCITY;
    hopperMotorSim.setIntegratedSensorRawPosition((int) sim_velocity_ticks_per_100_ms);
    hopperMotorSim.setIntegratedSensorVelocity((int) (getEncoderPosition() + Constants.LOOP_TIMESTEP_S * sim_velocity_ticks_per_100_ms));
    
  }
}

