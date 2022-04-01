/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import org.team2168.Constants;
import org.team2168.subsystems.Shooter.ShooterRPM;
import org.team2168.utils.Util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase implements Loggable {

  private static Limelight instance;
  private static double[] limelightdata = new double[3];
  private static double[] contourCorners = new double[8];
  private static NetworkTable networkTable;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry tcornxy; // gives x and y coordinates for corners of contour
  private static NetworkTableEntry ledMode;
  private static NetworkTableEntry camMode;
  private static NetworkTableEntry camtran;
  private static NetworkTableEntry pipeline;

  private static boolean variablesInstantiated = false;

  /*
   * Public so they can be used elsewhere; idk if this is good design but I need
   * it for right now because not every ll pipeline is correlated to a shooter
   * speed anymore and I don't want to overhaul the whole subsystem :/
   */
  // TODO create new pipelines
  public static final int PIPELINE_LAUNCHPAD_LINE = 0;
  public static final int PIPELINE_TARMAC_LINE = 1;
  public static final int PIPELINE_TERMINAL = 2;
  public static final int PIPELINE_BACK_TRENCH_RED = 3;
  public static final int PIPELINE_DRIVER_VIEW = 4;
  public static final int PIPELINE_DEFAULT_DRIVE = 5;
  public static final int PIPELINE_DRIVE_WITH_LIMELIGHT = 9;

  private static boolean isLimelightEnabled;

  private static int desiredCamMode = 1;
  private static LEDMode desiredLEDMode = LEDMode.PIPELINE;
  private static int desiredPipeline = 0;

  public double MAX_POSITIVE_ANGLE = 29.8;
  public double MIN_NEGATIVE_ANGLE = -29.8;

  public static double limelightMountAngle = 30.0;

  //Camera Controls (Use Enums to prevent invalid inputs)
  public enum LEDMode {
    PIPELINE(0),    // Use LED mode set in pipeline
    FORCE_OFF(1),   // Force LEDs off
    FORCE_BLINK(2), // Force LEDs to blink
    FORCE_ON(3);    // Force LEDs on

    LEDMode(int value) {
        this.val = value;
    }

    public int getCodeValue() {
        return val;
    }

    private int val;
  };

  private Limelight() {
      networkTable = NetworkTableInstance.getDefault().getTable("limelight");
      isLimelightEnabled = false;
      instantiateLocalVariables();
  }

  public int getDrivePipeline() {
    return PIPELINE_DEFAULT_DRIVE;
  }

  /**
   * @return an instance of the Intake Subsystem
   */
  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  /**
   * Sets the mode of the camera
   * 
   * @param doVisionProcessing Whether or not to do vision processing. If set to
   *                           false, the Limelight will act as a driver camera
   */
  public void enableVisionProcessing(boolean doVisionProcessing) {
    if (!connectionEstablished())
      return;

    else if (!variablesInstantiated)
      instantiateLocalVariables();

    camMode.setNumber(doVisionProcessing ? 0 : 1);
  }

  /**
   * Returns the target bearing
   * 
   * @return is a double from -27.0 to 27.0
   */
  @Log (name = "Target X (deg)", rowIndex = 2, columnIndex = 0)
  public double getPositionX() {
    return tx.getDouble(0.0);
  }

  /**
   * Returns the target bearing
   * 
   * @return is a double from -20.5 to 20.5 degrees
   */
  @Log (name = "Target Y (deg)", rowIndex = 2, columnIndex = 1)
  public double getPositionY() {
    return ty.getDouble(0.0);
  }

  /**
   * Returns the corners of target contour in a double array.
   */
  public double[] getCornerArray() {
    return tcornxy.getDoubleArray(contourCorners);
  }

  /**
   * Returns the horizontal average of all four corners of the target contour to find the center of the contour.
   * 
   */
  public double getCornerAvgX() {
    double cornerAvg = (getCornerArray()[0] + getCornerArray()[2] + getCornerArray()[4] + getCornerArray()[6]) / 4;
    // System.out.println(cornerAvg);
    return cornerAvg;
  }

  public void enableLimelight() {
    setCamMode(0);
    enableVisionProcessing(true);
    setLedMode(LEDMode.PIPELINE); // set to current pipeline setting
    isLimelightEnabled = true;
  }

  public void setPipeline(int pipelineNumber) {
    pipeline.setNumber(pipelineNumber);
    desiredPipeline = pipelineNumber;
  }

  @Log (name = "Estimated Distance", rowIndex = 3, columnIndex = 4)
  public double calcDistanceMeters() {
    return (Units.inchesToMeters(Constants.Heights.UPPER_HUB_HEIGHT_IN - Constants.Heights.ROBOT_LIMELIGHT_HEIGHT_IN))/Math.tan(limelightMountAngle + ty.getDouble(0));
  }

  @Log (name = "Active Pipeline", rowIndex = 1, columnIndex = 2)
  public int getPipeline() {
    if (this.connectionEstablished()) {
      return pipeline.getNumber(-1).intValue();
    } else {
      return -1;
    }
  }

  public void pauseLimelight() {
    setCamMode(1);
    enableVisionProcessing(false);
    setLedMode(LEDMode.FORCE_OFF); // force off
    isLimelightEnabled = false;

  }


  @Log (name = "Enabled?", rowIndex = 1, columnIndex = 0)
  public boolean isLimelightEnabled() {
    return isLimelightEnabled;
  }

  /**
   * Sets the LED mode
   * 
   * @param ledNumber is an int from 0 to 3
   *                  <ul>
   *                  <li>0 - use the LED Mode set in the current pipeline</li>
   *                  <li>1 - force off</li>
   *                  <li>2 - force blink</li>
   *                  <li>3 - force on</li>
   *                  </ul>
   */
  public void setLedMode(LEDMode ledNumber) {
    // setLedMode(ledNumber);
    desiredLEDMode = ledNumber;
  }

  @Log (name = "Connection?", rowIndex = 1, columnIndex = 1)
  private boolean connectionEstablished() {
    // return this.networkTable.containsKey("tx");
    return !(networkTable.getEntry("tx") == null);
  }

  private void instantiateLocalVariables() {
    // Variables to get data from Limelight
    tx = networkTable.getEntry("tx");
    ty = networkTable.getEntry("ty");
    ta = networkTable.getEntry("ta");
    tv = networkTable.getEntry("tv");
    camtran = networkTable.getEntry("camtran");

    // Variables to set data on Limelight
    ledMode = networkTable.getEntry("ledMode");
    camMode = networkTable.getEntry("camMode");
    pipeline = networkTable.getEntry("pipeline");
  }

  @Log(name = "Has Target?", rowIndex = 2, columnIndex = 0)
  public boolean hasTarget() {
    return tv.getBoolean(false);
  }

  /**
   * Returns the target's area, measured by the percentage of the frame it takes
   * up
   * 
   * @return is a double from 0.0 to 100.0
   */
  public double getTargetArea() {
      return ta.getDouble(0.0);
  }

  /**
   * Sets the limelight camera mode
   * 
   * @param camModeNumber an integer for the camera mode
   *                      <ul>
   *                      <li>0 - Vision processing</li>
   *                      <li>1 - Driver camera (More exposure, no vision
   *                      processing)</li>
   */
  private void setCamMode(int camModeNumber) {
    camMode.setNumber(camModeNumber);
    desiredCamMode = camModeNumber;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelightdata[0] = getPositionX();
    limelightdata[1] = getPositionY();
    limelightdata[2] = getTargetArea();

    if (!isLimelightEnabled) {
      pauseLimelight();
    }

    // Sets the camera controls
    ledMode.setNumber(desiredLEDMode.val);
    camMode.setNumber(desiredCamMode);
    pipeline.setNumber(desiredPipeline);
  }
}
