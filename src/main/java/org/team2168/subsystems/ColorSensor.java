// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ColorSensor extends SubsystemBase implements Loggable {
    private SerialPort serialPort;
    private static ColorSensor instance = null;
    private static final double DATA_THRESHOLD = 2.0;  // Time after which to consider data stale

    private Thread valueUpdateThread = new Thread(
            () -> {
                while (true) {
                    this.readSensor();
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        DriverStation.reportError("valueUpdateThread interrupted during sleep", e.getStackTrace());
                    }
                }
            });

    private static final SerialPort.Port SERIAL_PORT_PORT = SerialPort.Port.kOnboard; // port on the roborio
    // private static final int SERIAL_PORT_ADDRESS = 2; // just a place holder,
    // depends on what we give the teensy slave

//    private volatile int[] data = new int[4]; // {r, g, b}
    private volatile Color8Bit data = new Color8Bit(0, 0, 0);
    private volatile double timestamp = getTimeStampSeconds();

    private ColorSensor() {
        serialPort = new SerialPort(9600, SERIAL_PORT_PORT);
        serialPort.setReadBufferSize(4);
        serialPort.setWriteBufferSize(0);
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
     */
    public void readSensor() {
        serialPort.reset();
        byte[] serialOutput = null;

        while (serialOutput == null) {
            if (serialPort.getBytesReceived() >= 4) {
                serialOutput = serialPort.read(4);
                // convert values to integers
                var intValue = new int[serialOutput.length];
                for (int i = 0; i < serialOutput.length; i++)
                    intValue[i] = Byte.toUnsignedInt(serialOutput[i]); // Value on teensy will be unsigned int; byte is signed 2^7

                //
                if ((intValue[0] ^ intValue[1] ^ intValue[2]) != intValue[3]) {
                    System.out.println("Received garbled data from teensy!");
                    serialOutput = null;
                } else {
                    data = new Color8Bit(intValue[0], intValue[1], intValue[2]);
                }
            }
        }
        timestamp = getTimeStampSeconds();

    }


    @Log(name = "Color Sensor Alliance", methodName = "toString")
    public Alliance getColor() {
        if (data.red > data.blue)
            return Alliance.Red;
        else
            return Alliance.Blue;
    }

    @Log(name = "Is team color?")
    public boolean isTeamColor() {
        return DriverStation.getAlliance() == getColor();
    }

    @Log(name = "Is data stale?")
    public boolean isDataStale() {
        return getTimeSinceLastRead() > DATA_THRESHOLD;
    }

    @Log(name = "time since last read")
    public double getTimeSinceLastRead() {
        var currentTime = getTimeStampSeconds();
        return currentTime - timestamp;
    }

    public static double getTimeStampSeconds() {
        double timestamp = System.currentTimeMillis();
        return timestamp / 1000.0;
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

//         System.out.println(
//         String.format("R: %d G: %d B: %d", data.red, data.green, data.blue)
//         );

    }

    public Color8Bit getData() {
        return data;
    }
}
