# Guidelines for Contributing

- [Guidelines for Contributing](#guidelines-for-contributing)
  - [Documentation links:](#documentation-links)
  - [Software / Development Tools Setup](#software--development-tools-setup)
    - [Install WPILib:](#install-wpilib)
    - [Clone the Repo:](#clone-the-repo)
    - [Install Vendor dependencies:](#install-vendor-dependencies)
    - [Install Imaging Tools:](#install-imaging-tools)
    - [Building code:](#building-code)
    - [Testing your code in simulation:](#testing-your-code-in-simulation)
    - [Deploying code to the robot:](#deploying-code-to-the-robot)
  - [Code Style](#code-style)
    - [Formatting](#formatting)
    - [Variable Naming](#variable-naming)
      - [Command design](#command-design)
      - [Dependency Injection](#dependency-injection)
      - [Functional programming and DoubleSuppliers](#functional-programming-and-doublesuppliers)
  - [Git](#git)
    - [Branches](#branches)
      - [Branch naming scheme](#branch-naming-scheme)
    - [Committing](#committing)
      - [How to write commit messages](#how-to-write-commit-messages)
    - [Issues](#issues)
    - [Pull requests](#pull-requests)

## Documentation links:

We have a mirror of the 2022 javadocs for CTRE/REV/WPILib here: https://team2168.org/javadoc

* CTRE [read the docs](https://docs.ctre-phoenix.com/en/stable/), [examples code](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages)
* REV
  * REVLib [javadocs](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html)
  * SPARKMax [documentation](https://docs.revrobotics.com/sparkmax/), [example code](https://github.com/REVrobotics/SPARK-MAX-Examples)
  * Control system component [documentation](https://docs.revrobotics.com/), [PDH example code](https://github.com/REVrobotics/Power-Distribution-Hub-Examples)
    [Pneumatic Hub example code](https://github.com/REVrobotics/Pneumatic-Hub-Examples)

* WPILib [read the docs](https://docs.wpilib.org/en/stable/)
* Oblog [read the docs](https://oblog-docs.readthedocs.io/en/latest/) - for easy robot defined shuffleboard layouts

## Software / Development Tools Setup

### Install WPILib:

Download the latest release from: https://github.com/wpilibsuite/allwpilib/releases

Follow the installation steps here: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html

A shortcut should be added to your desktop named: `2022 WPILib VS Code`.

### Install PathPlanner:

If you plan to create autonomous routines, install pathplanner (Version 2022.1.0).

Download the `pathplanner-<platform>.zip` from: https://github.com/mjansen4857/pathplanner/releases/tag/v2022.1.0

**Do not download from the Windows/Mac App store!  These versions will automatically update.**

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

### Install Imaging Tools:

  * REV Hardware Client: https://www.revrobotics.com/software/
    * After installing run an update - yea even if you just installed it.
  * Follow the instructions here: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html
    This will include stuff like the RIO Imaging tool (to load firmware onto the roborio) and the DriverStation application.

### Building code:

  * From within VSCode 2022, Open the command pallet (Ctrl+Shift+P or click the WPI logo in the top right)
  * Type `Build` and select `WPILib: Build Robot Code`
  * If you code is syntactically correct (it can be compiled), you should see `BUILD SUCCESSFUL` displayed in the terminal.  
    IF there are compilation errors you should receive some diagnostic message identifying what the error is and in which file(s) it resides.

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

## Code Style

By in large, we do not enforce a styleguide so far.  If you would like to suggest one, feel free.  That said, there are still some guidelines to follow when writing.

### Formatting

Although we do not enforce any particular styleguide, there are still tools to keep your code orderly.  To format your code to remove irregularities, press ctrl+shift+p and select "Format Document".  This is a useful tool to normalize indentation and spacing, which makes your code more presentable and legible.

### Variable Naming

When naming variables, use camelCase as normal.  Please **do not** prepend member variable names with "_" (ex. `_driveMotor1` vs `driveMotor1`), as it is unnecessary and cluttered.  If you want to differentiate between local and member variables, be explicit by using [this](https://docs.oracle.com/javase/tutorial/java/javaOO/thiskey.html).

#### Command design

From previous years, I am trying to change the way we write commands to make them more general and useful.  The two main changes to accomplish this are the use of dependency injection for subsystems, and using functions to supply numerical inputs like joysticks rather than accessing them directly.

#### Dependency Injection

This change is not major.  All subsystems should be instantiated within RobotContainer, so when you instantiate a command using a subsystem, simply pass it in as a parameter.

#### Functional programming and DoubleSuppliers

Although I never went over it, we will be using DoubleSuppliers and other functions for more things this year.  The basic idea is that rather than passing in values (like passing in an int or double), you pass in a function which returns a value upon evaluation.  The most common type of function you'll use is `DoubleSupplier`.  If you have questions about how to use functions please ask, but I have included the most common usages below.

```java
// Constructor stub
public TankDrive(Drivetrain drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed)

// later on...
@Override
private void execute() {
  drivetrain.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
}


// methods in OI.java
public double getDriverJoystickX() // method stub
public double getDriverJoystickY() // method stub

// implementations in RobotContainer.configureButtonBindings
// method reference operator
new TankDrive(drivetrain, oi::getDriverJoystickX, oi::getDriverJoystickY);

// skip defining a method beforehand using a lambda
new TankDrive(drivetrain, () -> {
    double constantSpeed = 0.3;
    return constantSpeed;
  }, () -> {
    return constantSpeed = 0.3;
    return constantSpeed;
  }
);

// or more simply when you're only using 1 line
new TankDrive(drivetrain, () -> 0.3, () -> 0.3);
```

## Git

### Branches

Branches are a key way of dividing up tasks cleanly.  In order to use branches effectively, be sure to **only use a branch for one purpose**.  If you are adding a subsystem, make a branch for that subsystem and only that subsystem.  If you pause and work on another level, use a different branch.  This keeps the division between tasks clearer, and allows code to be pulled back into develop more granularly.

When creating branches, try to branch off of main when possible.  This provides for a common base between feature branches, which also makes pull requests easier.  There may be a reason to branch off of a feature branch instead, but be careful as to make sure this is actually the right decision.

**Always make pull requests when merging branches, even two feature branches.**  This makes the merge more public, and allows for code review and testing before the merge takes place.

#### Branch naming scheme

In general, try to keep branch names short and sweet, while also being specific enough so that someone knows what your branch is.  Delimit with underscores.  **Do not** use your name and initials, as this discourages collaboration and clutters the branch name.

*Ex.: If I was making a branch for level 5, a good name would be "level-5"*

### Committing

Committing is the primary way of saving your progress throughout development.  **Commit early and often, after every major task you complete.**

If you forget to commit things for a while and you have a lot of changes, still don't fret.  You can still break up the uncommitted changes into multiple commits to improve clarity.

#### How to write commit messages

Commit messages are the primary way of communicating what changes you have actually made.  Although it may seem annoying, taking the extra minute create a well written commit is important to convey your  changes to others, so someone can understand your commit without having to dig through the deltas themselves.

In addition, it may be helpful to include a file by file breakdown of the changes which were made.  This is optional, but could increase clarity in certain cases.

### Issues

Issues are a good way of tracking progress and reporting bugs.  If you find a bug/problem with the code, please open an issue, so it is visible to other people.  Include a brief description to go along with this.  If you an idea of where this bug is or how to fix it, also include that in the issue.

I would also invite you to use [keywords](https://docs.github.com/en/github/writing-on-github/working-with-advanced-formatting/using-keywords-in-issues-and-pull-requests) in your commits and pull requests to manage issues more concisely.

### Pull requests

Pull requests are used to pull completed changes into other branches.  **Please use them for all merges, not just merges into develop**.  If branches were used effectively, each pull request should only include changes of one kind (ex. pulling climber into  main would only add climber code to main).  Make sure to use a useful title, so it is clear what the pull request consists of.

If you think it would be helpful, you can also create a draft pull request before you're done with the feature.  This allows your code to be reviewed and commented upon more easily.  Once you have decided your code is in a working and finished state, you can convert it to a proper pull request before final review.

