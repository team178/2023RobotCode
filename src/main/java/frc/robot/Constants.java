// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
  }

  public static class DriveConstants {

    /*
     * Ports might be changed later, but here's how I plan to ID them
     *
     * LEFT:   _______ 
     *        /   -   \
     *       /         \
     *      /  10   11  \
     *      -------------
     * 
     * RIGHT:  _______ 
     *        /   -   \
     *       /         \
     *      /  13   12  \
     *      -------------
     * 
     * - Both viewed from the inside of the robot, back of the gearbox
     */

    public static final int kL1MotorPort = 10;
    public static final int kL2MotorPort = 11;
    public static final int kR1MotorPort = 12;
    public static final int kR2MotorPort = 13;

    public static final int kEncoderCPR = 2048; // Talon FX constant
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerRev = 2 * kWheelDiameter * Math.PI;
    public static final double kGearboxRatio = (40.0/34.0) * (22.0/14.0);

    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    public static final double kS = 1; //! Need to be tuned
    public static final double kV = 3;
    public static final double kA = 1;

    public static final double kPVel = 1;

  }
}
