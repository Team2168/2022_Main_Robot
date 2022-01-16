package org.team2168.PID.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSensor implements PIDSensorInterface
{

    // Target position and camera settings
    private NetworkTable networkTable;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry camtran;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry camMode;
    private NetworkTableEntry pipeline;

    private double currentPositionX;
    private double previousPositionX;

    private double currentPositionY;
    private double previousPositionY;

    private boolean variablesInstantiated;

    private double runTime;
    private int averagerSize;
    private double[] averagerArray;
    private int arrayPos = 0;

    private static final int LIMELIGHT_AVG_ENCODER_VAL = 0;

    /**
     * Default constructor
     */
    public LimelightSensor()
    {
        currentPositionX = 0.0;
        previousPositionX = 0.0;

        currentPositionY = 0.0;
        previousPositionY = 0.0;


        /**
         * Check networkTable to verify network connectivity of Limelight
         */

        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        variablesInstantiated = false;

        runTime = Timer.getFPGATimestamp();
        averagerSize = LIMELIGHT_AVG_ENCODER_VAL;
        averagerArray = new double[averagerSize];

        this.instantiateLocalVariables();

        if(this.connectionEstablished())
        {
            this.instantiateLocalVariables();
        }
        else
        {
            SmartDashboard.putBoolean("IsLimeLightPresent", false);
        }
    }

     /**
     * Returns the rate at which the target bearing changes
     */
    @Override
    public double getRate()
    {
        double executionTime = Timer.getFPGATimestamp() - runTime;
        double pos = getPosX();
        if(executionTime > 0) {
            putData((pos - previousPositionX) / executionTime);
        }

        runTime = Timer.getFPGATimestamp();
        previousPositionX = pos;
        return getAverage();
    }

    @Override
    public void reset()
    {
        // Does not apply
    }

    /**
     * Returns the target bearing
     * @return is a double from -27.0 to 27.0
     */
    @Override
    public double getPosX()
    {
        previousPositionX = currentPositionX;
        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            currentPositionX = tx.getDouble(0.0);
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            this.currentPositionX = tx.getDouble(0.0);
        }
        else
        {
            this.currentPositionX = 0.0;
        }

        return this.currentPositionX;
    }

    /**
     * Returns the target bearing
     * @return is a double from -20.5 to 20.5 degrees
     */
    @Override
    public double getPosY()
    {
        previousPositionY = currentPositionY;
        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            currentPositionY = ty.getDouble(0.0);
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            this.currentPositionY = ty.getDouble(0.0);
        }
        else
        {
            this.currentPositionY = 0.0;
        }

        return this.currentPositionY;
    }

    /**
     * Returns the target's area, measured by the percentage of the frame it takes up
     * @return is a double from 0.0 to 100.0
     */
    public double getTargetArea()
    {
        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            return ta.getDouble(0.0);
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return ta.getDouble(0.0);
        }
        else
        {
            return 0.0;
        }
    }

    /**
     * Returns the translation of the camera from the target
     * @return is an array of 6 doubles; gives the translation (x,y,z) and rotation (pitch,yaw,roll)
     */
    public double[] getCameraTranslation()
    {
        double[] defaultValues = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            return camtran.getDoubleArray(defaultValues);
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return camtran.getDoubleArray(defaultValues);
        }
        else
        {
            return defaultValues;
        }


    }

    /**
     * Sets the limelight camera mode
     * @param camModeNumber an integer for the camera mode
     * <ul>
     *  <li>0 - Vision processing </li>
     *  <li>1 - Driver camera (More exposure, no vision processing)</li>
     */
    private void setCamMode(int camModeNumber)
    {
        if(camModeNumber >= 0 && camModeNumber <= 2)
        {
            if (this.connectionEstablished() && this.variablesInstantiated)
            {
                camMode.setNumber(camModeNumber);
            }
            else if (this.connectionEstablished() && !this.variablesInstantiated)
            {
                this.instantiateLocalVariables();
                camMode.setNumber(camModeNumber);
            }
            // else
            // {
            //     System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            // }
        }
    }

    /**
     * Sets the mode of the camera
     * @param doVisionProcessing Whether or not to do vision processing.
     * If set to false, the Limelight will act as a driver camera
     */
    public void enableVisionProcessing(boolean doVisionProcessing) {
        if (!connectionEstablished())
            return;

        else if (!variablesInstantiated)
            instantiateLocalVariables();

        camMode.setNumber(doVisionProcessing? 0 : 1);
    }

    public int getCamMode()
    {
        if(this.connectionEstablished() && this.variablesInstantiated)
        {
            return camMode.getNumber(1).intValue();
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return camMode.getNumber(1).intValue();
        }
        else
        {
            //System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            return -1;
        }
    }

    /**
     * Sets the pipeline being used
     * @param pipelineNumber is an int from 0 to 9
     */
    public void setPipeline(int pipelineNumber)
    {
        if(pipelineNumber >= 0 && pipelineNumber <= 9)
        {
            if (this.connectionEstablished() && this.variablesInstantiated)
            {
                pipeline.setNumber(pipelineNumber);
            }
            else if (this.connectionEstablished() && !this.variablesInstantiated)
            {
                this.instantiateLocalVariables();
                pipeline.setNumber(pipelineNumber);
            }
            // else
            // {
            //     System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            // }
        }
    }

    /**
     * Returns the current pipeline number
     * @return is an int from 0 to 9
     */
    public int getPipeline()
    {
        if (this.connectionEstablished() && this.variablesInstantiated)
        {
            return pipeline.getNumber(0).intValue();
        }
        else if (this.connectionEstablished() && !this.variablesInstantiated)
        {
            this.instantiateLocalVariables();
            return pipeline.getNumber(0).intValue();
        }
        else
        {
            //System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            return -1;
        }

    }

    /**
     * Sets the LED mode
     * @param ledNumber is an int from 0 to 3
     * <ul>
     *   <li>0 - use the LED Mode set in the current pipeline</li>
     *   <li>1 - force off</li>
     *   <li>2 - force blink</li>
     *   <li>3 - force on</li>
     * </ul>
     */
    public void setLedMode(int ledNumber)
    {
        if(ledNumber >= 0 && ledNumber <= 3)
        {
            if (this.connectionEstablished() && this.variablesInstantiated)
            {
                ledMode.setNumber(ledNumber);
            }
            else if (this.connectionEstablished() && !this.variablesInstantiated)
            {
                this.instantiateLocalVariables();
                ledMode.setNumber(ledNumber);
            }
            // else
            // {
            //     System.out.println("Connection to Limelight not established. Check ethernet connectors.");
            // }
        }
    }

    private boolean connectionEstablished()
    {
        //return this.networkTable.containsKey("tx");
        return !(this.networkTable.getEntry("tx") == null);
    }

    private void instantiateLocalVariables()
    {
        SmartDashboard.putBoolean("IsLimeLightPresent", true);

        // Variables to get data from Limelight
        tx = networkTable.getEntry("tx");
        ta = networkTable.getEntry("ta");
        camtran = networkTable.getEntry("camtran");

        // Variables to set data on Limelight
        ledMode = networkTable.getEntry("ledMode");
        camMode = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");

        // Sets the camera controls
        ledMode.setNumber(0);
        camMode.setNumber(1);
        pipeline.setNumber(0);

        this.variablesInstantiated = true;
    }

      /**
	 * Puts data in to array to be averaged, hence the class name and method name.
	 * Its like magic but cooler.
	 *
	 * @param value the value being inserted into the array to be averaged.
	 */

    public synchronized void putData(double value) {
        averagerArray[arrayPos] = value;
        arrayPos++;

        if(arrayPos >= averagerSize) {
            // Is equal or greater to averagorSize because array is zero indexed. Rolls over index position
            arrayPos = 0;
        }
    }

    /**
	 * Returns average of last n values sent, as name says.
	 *
	 * @return the average
	 */
    public synchronized double getAverage() {
        double sum = 0;
        for(int i = 0; i < averagerSize; i++) {
            sum += averagerArray[i];
        }
        return sum / averagerSize;
    }


}
