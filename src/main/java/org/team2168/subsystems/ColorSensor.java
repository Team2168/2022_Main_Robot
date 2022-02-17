// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private SerialPort serialPort;
    private static ColorSensor instance = null;

    private static final SerialPort.Port SERIAL_PORT_PORT = SerialPort.Port.kOnboard; // port on the roborio
    //private static final int SERIAL_PORT_ADDRESS = 2; // just a place holder, depends on what we give the teensy slave


    int[] data = new int[3];  // {r, g, b}
    byte[] rawdata = new byte[3];
   // byte[] date = new byte[4];

    private ColorSensor() {
        serialPort = new SerialPort(9600, SERIAL_PORT_PORT); 
    }

    /**
     * Constructor for ColorSensor subsystem
     * 
     * @return instance of ColorSensor subsystem
     */
    public static ColorSensor getInstance() {
        if (instance == null)
            instance = new ColorSensor();
        return instance;
    }

    public byte[] readSensor() {
        serialPort.write(new byte[] {0x12}, 1);// once we write to teensy we receive data from teensy
        byte[] serialOutput = serialPort.read(3);
        for (int i = 0; i < serialOutput.length; i++)
            data[i] = Byte.toUnsignedInt(serialOutput[i]);  // Value on teensy will be unsigned int; byte is signed 2^7
        return rawdata = serialOutput;
    }

    // public static boolean validateSensor(byte[] data) {
    //      return (data[0] ^ data[1] ^ data[2]) == data[3];  // TODO; indexOutOfRange
    // }

    public Alliance getColor() {
        double total = getRed() + getGreen() + getBlue();
        double[] percentages = {(double) getRed()/total, (double) getGreen()/total, (double) getBlue()/total};
        System.out.println("Percentages: " + percentages);

        if (percentages[0] > 0.75)
            return Alliance.Red;
        if (percentages[1] > 0.4)
            if (percentages[2] > 0.2)
                return Alliance.Blue;
        return Alliance.Invalid;
        
    }

    public boolean isTeamColor() {
        return DriverStation.getAlliance() == getColor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
       // isConnected();
       // if (connected) {
            readSensor();
       // }
       // System.out.print(connected);
       SmartDashboard.putNumber("R", getRed());
       SmartDashboard.putNumber("G", getGreen());
       SmartDashboard.putNumber("B", getBlue());
       SmartDashboard.putString("Computed output", getColor().toString());
       System.out.print(" R:"+getRed());
       System.out.print(" G:"+getGreen());
       System.out.print(" B:"+getBlue());
    }

    public int getRed() {
        try {
            return data[0];
        } catch (ArrayIndexOutOfBoundsException e) {
            return 0;
        }
    }

    public int getGreen() {
        try {
            return data[1];
        } catch (ArrayIndexOutOfBoundsException e) {
            return 0;
        }
    }

    public int getBlue() {
        try {
            return data[2];
        } catch (ArrayIndexOutOfBoundsException e) {
            return 0;
        }
    }
}
