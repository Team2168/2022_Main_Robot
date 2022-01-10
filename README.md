# 2022_Main_Robot

## Software Setup

### Install WPILib:

Download the latest release from: https://github.com/wpilibsuite/allwpilib/releases

Follow the installation steps here: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html

A shortcut should be added to your desktop named: `2022 WPILib VS Code`.

### Clone the Repo:

From `git bash`:
  * Change directories (`cd`) to where you want a copy of the project created. (e.g. C:\Users\<username>\Documents\!FRC\git\ or wherever is convenient/memorable for you) 
  * `git clone https://github.com/Team2168/2022_Main_Robot.git`


### Install Vendor dependencies:

* CTRE: Support for F500s, Talon Motor controllers, CANCoders, Pigeon IMU, etc.

  Download and install the latest release of the pheonix framework here: https://github.com/CrossTheRoadElec/Phoenix-Releases/releases
  
* REV Robotics: for the SPARK MAX
  Should be added already, vendor dep URL that is currently working:
  https://rev-robotics-software-metadata.netlify.app/REVLib.json


For more information see here: https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html

### Install NI Tools:

Follow the instructions here: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html

This will include stuff like the RIO Imaging tool (to load firmware onto the roborio) and the DriverStation application.

### Building code:

  * From within VSCode 2022, Open the command pallet (Ctrl+Shift+P or click the WPI logo in the top right)
  * Type `Build` and select `WPILib: Build Robot Code`
  * If you code is syntactically correct (it can be compiled), you should see `BUILD SUCCESSFUL` displayed in the terminal.  
    IF there are compilation errors you should recieve some diagnostic message identifying what the error is and in which file(s) it resides.

### Testing your code in simulation:

  * Being able to compile code doesn't mean it will do what you want.
    It's important to verify that your code does what it expects.  
    This can be challenging without the real robot & real hardware, but WPILib now ships with simulation functionality,
    allowing you to find runtime errors without having access to a physical robot.

  * From within VSCode 2022, Open the command pallet (Ctrl+Shift+P or click the WPI logo in the top right)
  * Type `Simulate` and select `WPILib: Simulate Robot Code`
  
 
### Deploying code to the robot:
  If your code compiles & runs in simulation, and you're ready to deploy code to the robot: 

  * Join the WiFi network for the robot you want to deploy code to
    * It's roborio needs 2022 compatible firmware flashed

  * From within VSCode 2022, Open the command pallet (Ctrl+Shift+P or click the WPI logo in the top right)
  * Type `Deploy` and select `WPILib: Deploy Robot Code`
  * The code will build, be uploaded to the roborio, and executed
  
  * Before running your code:
     * clear the area of people, tools, or debris that might be at risk of being hit
     * talk to people in the area, make sure they know what you plan on doing / what the robot may do.
   * Open the DriverStation
     * Double check you're in the right game mode Practice/Auto/Teloperated
     * Audibly identify you're enabling the robot (yell `ENABLING - WATCH YOUR FACE`)
     * Enable the robot

