// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import java.util.concurrent.CountDownLatch;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private SerialPort serialPort;
    private static ColorSensor instance = null;

    private Thread valueUpdateThread;

    private static final SerialPort.Port SERIAL_PORT_PORT = SerialPort.Port.kOnboard; // port on the roborio
    // private static final int SERIAL_PORT_ADDRESS = 2; // just a place holder,
    // depends on what we give the teensy slave

    volatile int[] data = new int[3]; // {r, g, b}
    // byte[] date = new byte[4];

    private ColorSensor() {
        serialPort = new SerialPort(9600, SERIAL_PORT_PORT);
        serialPort.setReadBufferSize(3);
        serialPort.setWriteBufferSize(1);
        valueUpdateThread = new Thread(() -> {
            while (true) {
                try {
                    this.readSensor();
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                }
            }
        });
        valueUpdateThread.start();
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

    /**
     * A method to get rgb values from the teensy.
     * 
     * This will block until it recieves values, so be careful with where you use
     * it.
     * 
     * @return bytes from teensy
     */
    public byte[] readSensor() {
        serialPort.reset();
        serialPort.write(new byte[] { 0x12 }, 1);

        var previousWrite = Timer.getFPGATimestamp();
        while (serialPort.getBytesReceived() < 3) {
            var currentTimestamp = Timer.getFPGATimestamp();
            if ((currentTimestamp - previousWrite) > 1.0) {  // Write once a second
                serialPort.write(new byte[] { 0x12 }, 1);
                previousWrite = currentTimestamp;
            }

        }

        byte[] serialOutput = serialPort.read(3);

        // convert values to integers
        for (int i = 0; i < serialOutput.length; i++)
            data[i] = Byte.toUnsignedInt(serialOutput[i]); // Value on teensy will be unsigned int; byte is signed 2^7
        return serialOutput;
    }

    // public static boolean validateSensor(byte[] data) {
    // return (data[0] ^ data[1] ^ data[2]) == data[3]; // TODO; indexOutOfRange
    // }

    public Alliance getColor() {
        if (getRed() > getBlue())
            return Alliance.Red;
        else
            return Alliance.Blue;
    }

    public boolean isTeamColor() {
        return DriverStation.getAlliance() == getColor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

            SmartDashboard.putNumber("R", getRed());
            SmartDashboard.putNumber("G", getGreen());
            SmartDashboard.putNumber("B", getBlue());
            SmartDashboard.putString("Computed output", getColor().toString());


        // System.out.println(
        //     String.format("R: %d G: %d B: %d", getRed(), getGreen(), getBlue())
        // );

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
