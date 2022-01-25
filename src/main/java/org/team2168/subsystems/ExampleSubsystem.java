// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ExampleSubsystem extends SubsystemBase implements Loggable {


  @Log(rowIndex = 0, columnIndex = 0, width = 1, height = 1)
  int i = 0;
  @Log.Graph(rowIndex = 0, columnIndex = 1, width = 2, height = 2, visibleTime = 5)
  double speed = 0.0;

  double kP = 0.0, kI = 0.0, kD = 0.0;
  double setpoint = 0.0;
  @Log.Graph(rowIndex = 2, columnIndex = 4, width = 2, height = 2, visibleTime = 5)
  double encoder = 0.0;

  PIDController exampleController = new PIDController(kP, kI, kD);

  private static DoubleSolenoid test_actuator = new DoubleSolenoid(Constants.PneumaticsDevices.MODULE_TYPE, Constants.PneumaticsDevices.TEST_EXTEND, Constants.PneumaticsDevices.TEST_RETRACT);

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    i+=1;
    encoder = exampleController.calculate(encoder, setpoint);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Config(rowIndex = 0, columnIndex = 5, width = 1, height = 1)
  public void set_setpoint(double input) {
    setpoint = input;
  }

  public void extend() {
    test_actuator.set(Value.kForward);
  }

  public void retract() {
    test_actuator.set(Value.kReverse);
  }

  @Config(rowIndex = 0, columnIndex = 4, width = 1, height = 2)
  public void setPID(double p, double i, double d) {
    exampleController.setPID(p, i, d);
  } 
}
