// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import org.team2168.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final boolean _INDEXER_MOTOR_REVERSED = false;
  private TalonSRX _srx_motor;
  private static DigitalInput entranceLineBreak;
  private static DigitalInput exitLineBreak;
  private static Indexer _instance = null;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 30; //amp
  private final double TRIGGER_THRESHOLD_TIME = 0.02; //s

  public Indexer() {
    entranceLineBreak = new DigitalInput(Constants.ENTRANCE_LINE_BREAK);
    exitLineBreak = new DigitalInput(Constants.EXIT_LINE_BREAK);

    _srx_motor = new TalonSRX(Constants.INDEXER_MOTOR_PDP);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    _srx_motor.configFactoryDefault();
    _srx_motor.configSupplyCurrentLimit(talonCurrentLimit);

    _srx_motor.setNeutralMode(NeutralMode.Brake);
    _srx_motor.setInverted(true); //output polarity is apparently reversed between SRX and SPARK MAX?
    _srx_motor.configNeutralDeadband(0.05);

  }

  public static Indexer getInstance(){
    if(_instance == null){
      _instance = new Indexer();
    }
    return _instance;
  }  

  public void drive(double speed) {
      if(_INDEXER_MOTOR_REVERSED) {
        speed = speed * -1;
      }
      _srx_motor.set(ControlMode.PercentOutput, speed);
    }
  

  public double isBallEnteringDashboard() {
    if(!entranceLineBreak.get()) {
      return 1.0;
    }
    else {
      return 0.0;
    }
    //return !entranceLineBreak.get();
  }

  public boolean isBallEntering() {
    return !entranceLineBreak.get();
  }

  public double isBallExitingDashboard() {
    if(!exitLineBreak.get()) {
      return 1.0;
    }
    else {
      return 0.0;
    }
  }

  public boolean isBallExiting() {
    return !exitLineBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
