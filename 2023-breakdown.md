# Subsystems
## [Arm](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/subsystems/Arm.java)
This subsystem controls the entire arm. We orignally had it split into two subsystems, seperate for the upper and lower arm, but they were nearly identical, so we consolidated them into one arm.

This subsystem controls it's motor outputs using it's [periodic loop](https://github.com/team178/2023RobotCode/blob/b99670a05940945b707624d334401d644e1c6502/src/main/java/frc/robot/subsystems/Arm.java#L176-L226). Feedforward and PID calculations are made, and motor output voltages are set. We also included logic to stop the arm motors if the arms try to move downward and the limitswitches are pressed, to keep them from driving downwards really hard if something went wrong. This is about all those limit switches do.

The PID loop uses setpoints provided by the [`ArmPosition`](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/commands/ArmPosition.java) enum under the commands folder. Each value of the enum contains a pair of values, each being the measure of an arm position from the encoders in radians.

```java
public enum ArmPosition {
    // Arm positions in radians
    HOLD,
    HOME (5.407518, 5.999611),
    SUBSTATION(4.093430, 5.156882),
    HIGH (2.976432, 3.154062),
    LOW (3.587507, 4.466632),
    BACK (5.407518, 2.951937);

    public double lower;
    public double upper;

    private ArmPosition() {}

    private ArmPosition(double lower, double upper) {
        this.lower = lower;
        this.upper = upper;
    }
}
```

The setpoint that the arm tries to reach can be changed using the [`setPosition`](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/subsystems/Arm.java#L119-L129) funcitonal command. By default, it tries to reach the `HOME` setpoint. We originally had a `HOLD` value in the enum, with no setpoint measurements, so we could create a special case where the arm would just try to hold it's current position, but we ran into issues with that and the encoders, so we didn't pursue it much further and just commented out the code.

To be able to test the arm encoders in the beginning, we implemented a `Mechanism2d` display. This allowed us to see a visual of the arm, and see where the encoders think the arm is. This didn't have much use after that.

### The Death Flip

Speaking of encoder measurements, we had some ***fun*** with that.

We used the REV Throughbore encoders for measuring our arm's motion this year. NEOs have a built-in encoder. All brushless motors will have some kind of sensor in them, usually a Hall-Effect sensor, to run the motor. But a perk of the REV Throughbore encoders is a mode they have, called Absolute mode. An absolute encoder is an encoder that doesn't just measure it's change in position since it was reset, it knows it's zero and how far it's moved from that position *at all times*. This means even when the robot has turned off, or the encoder is reset, it still knows exactly what position it's in. This is a lot safer for the arm than hoping limit switches don't get pressed by accident and changing the position of our zero.

Now, that didn't stop us from having code problems with it that cause fairly large issues. We discovered at drive practice one day that the arm sometimes like to do something we called the 'death flip'. This is where the arm, in most cases the upper arm, suddenly decides it wants to drive itself backwards as fast as it can, and it won't stop until it either releases the magic smoke from the motor, we disable it, or the gearbox gives way and lets the screws holding it in snap the mounting holes off.

We tried many things to solve this, and finally, the meeting before DCMP, we found the issues.

#### What went wrong?

The encoder is plugged into the RoboRIO using one of the DIO Ports. In the code, we use a `DutyCycleEncoder` class's `getDistance()` method to get the reading from the encoder. We supply the class with a 'distance-per-rotation' value of 2π(2 times PI). So, this method got us the measurement we needed. Now, what we didn't expect, is for this class to account for rollover.

Rollover is when the encoder makes one full rotation. In absolute mode, once it reaches that full rotation, it's actual output resets back to zero. The same thing happens going the other way. This class acounts for that, so when you go over one rotation, instead of going back to zero, it adds +1 to an internal counter, so you can go over 1 rotation.

Turns out that it does this in a way that if the encoder is very briefly disconnected, it thinks it went a full rotation.

TLDR, the DIO connection was loose. It kept disconnecting very briefly, usually upon an impact, and the code thinks the encoder when a full rotation. So suddenly, while it's at 5.9 radians and trying to get to 5.4 radians, the encoder reading suddenly becomes 11.5 radians and it think it's somewhere it isn't.

#### How did we fix it?

The first thing we did was tape down the DIO connectors with some electrical tape. I've seen other teams use hot glue to keep their PWM connectors down, which we should probably start doing.

In the code, to prevent the rollover issue, we stopped using the `getDistance()` method, and started using the `getAbsolutePosition()` method in our `Arm` class, which does not account for rollover. Note that we had to adjust the lower encoder so that the arm positions didn't have negative values. The `(1/6)` we recently discovered does nothing, since Java does integer division that way (it's just 0).

```java
public double getLowerPosition() {
    return Units.rotationsToRadians(m_lowerEncoder.getAbsolutePosition() - m_lowerEncoder.getPositionOffset() - (1/6));
}

public double getUpperPosition() {
    return Units.rotationsToRadians(m_upperEncoder.getAbsolutePosition() - m_upperEncoder.getPositionOffset());
}
```

### What could be done better

1. **Why did we use radians?**

	We could have just used the raw rotation measurements instead of all this conversion nonsense we had to go through.

2. **`ArmPosition` shouldn't be the commands folder**

	Just a bit petty, but it doesn't really make sense being in there even though I put it there.

1. **More PID and FF turning.**

	We didn't really dig too much into this for the arm. We learned very quickly that SysID sucks a lot, and we ended up taking the values from it and manually tweaking them until they gave use good enough values. That seems to be how a lot of people are doing it. 
	
	Our P constant, `3`? We just picked that number and it happened to work really well, and whenever we changed it it didn't work as well, so we just left it.

## [Drivetrain](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/subsystems/Drivetrain.java)

A lot of the drivetrain code this year was an attempt to make a better drivetrain code and it ended up looking really scary and messy. But what it actually does it a whole lot simpler than it appears, because Java sucks and makes things really abstracted and scary.

All most of that code does is the same thing we were doing with the arm, using a PID and FeedForward loop to control the speeds of the wheels. Because we were using Falcons and we're not going to pay the subscription for Pheonix PRO, we convert our Falcon's encoder readings to meters or meters-per-second, and put that into the loop to control their speed. 

Because the falcons were behind the gearbox, we had to do some maths to figure out how far the actual output shaft went. To do so, we multiply the number of motor rotations by the gear ratio to get the number of wheel rotations.

```java
double wheelRotations = motorRotations * DriveConstants.kGearboxRatio;
```

We didn't put too much effort into tuning the PID and FF values for it because it was close enough and we had other things to figure out.

This kind of functionality doesn't do much for teleop modes. It's more useful when doing autonomous. Being able to set and control the wheel speeds using physical measurements makes it a lot easier to make the bot follow a set path.

For our teleop drive methods, we have two in here. The first one is standard arcade drive, but it looks a bit fancier since we implement a custom kinematics class.
```java
public void arcadeDrive(double forward, double rot) {
	var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(
    new ChassisSpeeds(-forward, 0.0, -rot));
	setWheelSpeeds(wheelSpeeds);
}
```

The other drive method is called Cheesy Drive. It's a drive method taken from the code of Team 254, the Cheesy Poofs. It is similar to another drive method called Curvature drive, in which instead of just setting the x and z movement speeds, it offsets the speeds of each side of the drivetrain to change the curvature of the robot's path. In summary, it drives more like a car, allowing more ease in control when driving at higher speeds.

The difference with cheesy drive is that it uses some more complex math, and also includes a quick-turn feature, that allows arcade style driving when pressing a trigger, so that while you can still have the high-speed turning benefit of curvature drive, you can still switch to arcade and be able to turn on the spot at lower speeds.

Why aren't we using it? Because for some reason the Falcons overheat when using it. We have no idea why. Plus, David was doing better with arcade drive.

### Pose Estimation

One thing we tried for autonomous this year was pose estimation. We implemented this as part of the drivetrain.

Pose estimation is the robot using sensors such as gyros, encoders, and readings from the Limelight to estimate it's position on the field. This because especially useful with the new AprilTags this year.

In the periodic method of the drivetrain, we update the pose estimation class. We provide the encoder and gyro measurements, and we also use a pose generated by the Limelight.

The Limelight has it's own pose esitmation built in which uses the AprilTags around the field. We grab this pose from NetworkTables, and plug it into our estimatior.

```java
if (LimelightHelpers.getTV("limelight")) {
    // The pose from limelight for some reason has it's orign in the middle of the
    // field instead
    // of the bottom left like the WPILib pose estimator, so we have to account for
    // that
    Pose2d botpose = new Pose2d(
        botposeEntry[0] + FieldConstants.kFieldLength / 2,
        botposeEntry[1] + FieldConstants.kFieldWidth / 2,
        Rotation2d.fromDegrees(botposeEntry[5]));

    m_poseEstimator.addVisionMeasurement(
        botpose,
        Timer.getFPGATimestamp() - (botposeEntry[6]/1000));
}
```

This pose is then averaged with the other sensor measurements to create a final estimated pose. But, the Limelight's estimations were fairly sparatic, which is common with a very shakey camera and image recognition. So, to fix this, we provided what is called a trust matrix. This tells the estimator how much to trust each measurement from the limelight, the X, Y, and rotation measurements.

```java
private static Matrix<N3, N1> m_createVisionTrustMatrix() {
    Matrix<N3, N1> matrix = new Matrix<N3, N1>(N3.instance, N1.instance);
    matrix.set(0, 0, 5); // X
    matrix.set(1, 0, 5); // Y
    matrix.set(2, 0, 5); // Theta
    return matrix;
}
```

I'm going to go on a bit of a rant here and say that this `Matrix` class is the reason I hate Java...

<details>
<summary>Click here for me screaming</summary>
IT'S LITERALLY JUST AN ARRAY. WHY COULDN'T THIS JUST BE A SINGLE METHOD ON THE POSE ESTIMATOR???? WHY DO I NEED TO MAKE A WHOLE OBJECT??? AND TO MAKE IT WORSE IT'S SIZE IS DEFINED IN THE &lt&gt. BUT WHY. WHYYYY DID THEY DECIDE TO MAKE INDIVIDUAL CLASSES FOR EVERY NUMBER FROM 0 TO 20. THEY LITERALLY JUST REPRESENT THE NUMBER, THAT'S ALL THEY DO. WHY. WHY DOES THIS EXIST.

https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/numbers/package-summary.html
</details>

Anyway.

### What could be better
1. Tuning PID and FF values better
2. Tuning Limelight and pose estimation
	We had more important issues to fix this year that took priority over figuring out optimal Limelight settings that might make it's estimation more stable.

## [Claw](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/subsystems/Claw.java)
The claw is by far the most simple of the subsystems. It's so short that I can paste it here.
```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClawConstants.kChannel);
    private DigitalInput m_photosensor = new DigitalInput(2);

    public Claw() {
        m_solenoid.set(false);
    }

    public boolean getPhotosensor() {
        return !m_photosensor.get();
    }

    public CommandBase toggle() {
        return Commands.runOnce(() -> m_solenoid.toggle());
    }

    public Command close() {
        return Commands.runOnce(() -> m_solenoid.set(false));
    }

    public Command open() {
        return Commands.runOnce(() -> m_solenoid.set(true));
    }

    public void periodic() {
        SmartDashboard.putBoolean("photosensor", getPhotosensor());
    }

}
```

This subsystem manages only the solenoid for controlling the claw, and also holds the outputs for the Photosensor we use to detect pieces in front of the claw.

# Autonomous

The original plan for Autonomous was to implement and use [PathPlanner](https://github.com/mjansen4857/pathplanner). PathPlanner is an application that allows you to draw and make a path for the robot to follow, which can then be deployed to the robot and turned into robot motion by using PathPlannerLib, a Java library for path planner. But, we ran into some issues, and didn't fully implement the library. A bit of our auto code is more of a bodge this year than I hoped.

## Auto commands

The [`Autos`](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/commands/Autos.java) class is provided in the Command robot template from WPILib. It serves as a utility class, and shouldn't be instantiated. It contains static methods for functional commands, or other utilities relating to autonomous.

For us, this class served as a central point for all of our auto commands, containing our auto selector. We created a static `SendableChooser` in this class, and ran the `initAutoChooser` method from our `RobotContainer` constructor. This populated our selector with all of our commands. The selected command can then be obtained on autonmous init by using the `getSelectedAuto` command.

```java
    public static void initAutoChooser(Arm arm, Claw claw, Drivetrain drivetrain) {
        autoChooser.setDefaultOption("None", new AutoCommand());

        autoChooser.addOption("SubConeLeave", new SubConeLeave(arm, claw, drivetrain));
        autoChooser.addOption("SubConeCube", new SubConeCube(arm, claw, drivetrain));
        autoChooser.addOption("SubConeCubeCharge", new SubConeCubeCharge(arm, claw, drivetrain));

        autoChooser.addOption("MidCubeCharge",/*parker wuz here*/ new MidCubeCharge(arm, claw, drivetrain));
        autoChooser.addOption("MidCubeChargeOverBack", new MidCubeChargeOverBack(arm, claw, drivetrain));

        autoChooser.addOption("BumpConeLeave", new BumpConeLeave(arm, claw, drivetrain));
        autoChooser.addOption("BumpConeCube", new BumpConeCube(arm, claw, drivetrain));

        Shuffleboard.getTab("Autos")
            .add("Auto", autoChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(9, 1);
    }
```

There is only one auto command that is written in the `Autos` class, which is the `placeHigh` command.

The rest of the commands are found under `commands/auto`. There is a base class, called `AutoCommand`, which we extend off of for the rest of the commands. This defines a method that all of these classes should have, which is `getStartPosition`. This method allows us to fetch the starting position of the robot, and we can give it to our either our pose estimator, so it knows exactly where it's starting, or to a Shuffleboard widget to display the correct starting position for that auto.

## Trajectories

Each command loads it's trajectories, or paths generated using pathplanner, in it's constructor. We'll use `MidCubeCharge` as an example.

```java
getOnCharge = new AutoTrajectoryPair(PathPlanner.loadPath("MidGetOnCharge", new PathConstraints(1.0, 5), true));
```

Here, we load a path called `"MidGetOnCharge"`, which drives the robot backwards to get it up on the charge station so it can start to level itself. We provide a `PathContraints` object, which tells pathplanner how fast to drive the trajectory, and the final `true` tells it that the robot should drive the path backwards (very important).

Now, we do something that is *not* what PathPlannerLib really wants us to do, and we put the newly generated path into a class called [`AutoTrajectoryPair`](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/commands/auto/AutoTrajectoryPair.java). This class was a solution to an issue that was introduced this year because of the field.

This year, unlike previous years, the field is *mirrored*. This means that while in previous years, you could generate one path and just rotate it, or just use the same path and not use Limelight in your pose estimation, we can't do that now. We have to completely mirror the field.

Unfortunately, PathPlannerLib only took us so far. It mirrors the path, but only over the X axis. This would work when not using Limelight, but when the robot is going to be guessing where it is on the field *exactly*, we can't have it speeding across the field to balance on the other team's charge station. So, we had to abandon PathPlannerLib's trajectory following commands. Instead, we wrote our own method to mirror a trajectory across the Y axis instead.

```java
	/*
    * Mirrors the provided trajectory across the field
    */
    public static Trajectory mirrorTrajectory(Trajectory traj) {

        // Calculate the transformed first pose.
        List<State> newStates = new ArrayList<>();

        for (var state : traj.getStates()) {
            newStates.add(
                new State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(
                        FieldConstants.kFieldLength - state.poseMeters.getX(),
                        state.poseMeters.getY(),
                        state.poseMeters.getRotation().times(-1).plus(
                        new Rotation2d(Units.degreesToRadians(180))
                        )
                    ),
            state.curvatureRadPerMeter));
        }

        return new Trajectory(newStates);
    }
```

Now, we ran into a new problem. Because our commands and trajectories are created and loaded the robot's constructor, the `Trajectory` objects created cannot be changed easily. So, we settled on the solution of generating two `Trajectory` objects. One would be for the Blue alliance, and a mirrored one for the Red alliance. Then, when the robot is told to drive a trajectory, it fetches the correct trajectory depending on which alliance it's on.

We ran into a bit of a problem with that though, being that the `RamseteCommand` we use to follow the trajectories were also created by calling the init method in the constructor. So, we had to make our own, and so [`DriveTrajectory`](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/commands/DriveTrajectory.java) was born. It's basically a copy paste of `RamseteCommand`, but we added a little change. We created a new trajectory that can take a supplier, or a Java method that can return a `Trajectory` object. Then, we can run a function to decide what trajectory to use when the command is schedule to run instead of when it's created.

```java
// `getAllianceTrajectory` is called when this command starts to run instead
// of when it's created
new DriveTrajectory(drivetrain, getOnCharge::getAllianceTrajectory)
```

[`AutoTrajectoryPair`](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/commands/auto/AutoTrajectoryPair.java) was a way to neaten this up, and does what I've explained previously: Creating two trajectories, then it contains a method which we pass to `DriveTrajectory`.

```java
public class AutoTrajectoryPair {
    private Trajectory redPath;
    private Trajectory bluePath;
    
    public AutoTrajectoryPair(Trajectory bluePath) {
        this.bluePath = bluePath;
        redPath = Autos.mirrorTrajectory(bluePath);
    }

    public AutoTrajectoryPair(Trajectory path, Alliance alliance) {
        if(alliance.equals(Alliance.Red)) {
            redPath = path;
            bluePath = Autos.mirrorTrajectory(redPath);
        } else {
            bluePath = path;
            redPath = Autos.mirrorTrajectory(bluePath);
        }
    }

    public Trajectory getAllianceTrajectory() {
        return DriverStation.getAlliance().equals(Alliance.Red) ? redPath : bluePath;
    }
}
```

We didn't spend much time figuring out the other features of PathPlanner once we had to mirror the field and abandon PathPlannerLib's own trajectory commands, but they do have some features that seem useful, such as markers that can be created in the GUI and made to run commands at those points.

PathPlanner and other trajectory following methods are much better on holonomic drivetrains, such as swerve and meccanum. Tank drive robots have a bit more trouble following these paths accurately at bearable speeds. We had to do a lot of tweaking with our paths, and looking at some of them, they might look like they're messed up, or off. But, that's just what we had to make to get the robot to move to the right location.

## [`placeHigh`](https://github.com/team178/2023RobotCode/blob/b99670a05940945b707624d334401d644e1c6502/src/main/java/frc/robot/commands/Autos.java#L119-L127)

This is a common command used by most of our other commands. It's in the `Autos` class partially because it's simple enough to not need it's own class, but also mostly because we put it there first and didn't want to move it.

This command is a simple sequence of other commands to set the arm position and open and close the claw.

```java
  public static Command placeHigh(Arm arm, Claw claw) {
    return Commands.sequence(
        claw.close(),
        arm.setPosition(ArmPosition.HIGH),
        new WaitCommand(1.5),
        claw.open(),
        new WaitCommand(0.3),
        arm.setPosition(ArmPosition.HOME));
  }
```


## [`DriveUntilLevel`](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/commands/DriveUntilLevel.java)

This one was created between WNE and Hartford. Leveling the charge station by just driving on it and hoping it would be in a good-enough position to level it wasn't going to cut it. So, I broke our the Gyro box and taped an analog gyro to the side of the RIO so we could get an angle of the robot when it's on the charge station, then made a simple bang-bang command called `DriveUntilLevel`.

```java
  @Override
  public void execute() {
    if (-6 > m_drivetrain.getLevelHeading()) {
      m_drivetrain.arcadeDrive(-m_speed, 0);
    } else if (m_drivetrain.getLevelHeading() > 6) {
      m_drivetrain.arcadeDrive(m_speed, 0);
    } else {
      m_drivetrain.setWheelSpeeds(0, 0);
    }
  }
```

If the robot is leaning forward, drive backwards slowly. If it's leaning backwards, drive forward sowly. If it's within a range in which the charge station could level itself, don't move. Pretty simple. Slow enough that it makes you anxious during those 15 seconds, but it works very well.

# Other things

## [Lights](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/subsystems/Lights.java)

Lights this year were ran off the RoboRIO instead of a seperate Arduino controller like previous years. They were programmed to change between the alliance color and the color of the gamepiece the driver's wanted when a button on the controller was pressed.

< insert things from people who actually wrote this code >

## [Combo](https://github.com/team178/2023RobotCode/blob/main/src/main/java/frc/robot/utils/Combo.java)
"Hey, so we have an arcade cabinet controller this year, right? So we can make combo moves?"

## m_
"Why do you keep putting 'm_' before all your variable names?"

"It's because... uh.... [some kind of code style thing](https://en.wikipedia.org/wiki/Hungarian_notation)"
