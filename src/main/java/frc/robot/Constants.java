// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * CAN ID "groups":
 * I'm organizing each subsystem to be organized into groups.
 * 
 * 1x - Drivetrain
 * 2x - Arm
 */
public final class Constants {
  public static class FieldConstants {
    public static final double kFieldWidth = Units.inchesToMeters((26 * 12) + 3.5);
    public static final double kFieldLength = Units.inchesToMeters((54 * 12) + 3.25);
  }


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
    public static final int kAltDriverControlPort = 2;
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
    public static final double kEncoderDistancePerRev = kWheelDiameter * Math.PI;
    public static final double kGearboxRatio = (34.0/40.0) * (14.0/50.0);

    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    public static final double kMaxSpeedMetersPerSecond = 6;
    public static final double kMaxRotationSpeedMetersPerSecond = 6;

    public static final double kS = 0.50892;
    public static final double kV = 0.28201;
    public static final double kA = 1.1083;

    public static final double kPVel = 3.1285;

    // This object is why I hate Java - Patrick
    public static final Matrix<N3, N1> kVisionTrustMatrix = m_createVisionTrustMatrix();

    private static Matrix<N3, N1> m_createVisionTrustMatrix() {
      Matrix<N3, N1> matrix = new Matrix<N3, N1>(N3.instance, N1.instance);
      matrix.set(0, 0, 5); // X
      matrix.set(1, 0, 5); // Y
      matrix.set(2, 0, 5); // Theta
      return matrix;
    }

    public static final double slowModeMult = 0.2;
    public static final double defaultSpeedMult = 0.6;
  }

  public static class ClawConstants {
    public static int kChannel = 6;
    public static int kUltrasonicPort = 3;
  }
  
  public static class ArmConstants {
    public static final int kLowerMotorPort = 20;
    public static final int kUpperMotorPort = 21;

    public static final int kLowerHomePort = 0;
    public static final int kUpperHomePort = 1;

    public static final int kLowerArmEncoder = 7;
    public static final int kUpperArmEncoder = 8;
    
    // Upper arm

    public static double kUpperArmP = 3;
    public static double kUpperSVolts = 0.66617;
    public static double kUpperGVolts = 0.085621;
    public static double kUpperVVoltSecPerRad = 1.944;
    public static double kUpperVVoltSecPerRadSquared = 0.046416;
    
    // Lower arm 
    
    public static double kLowerArmP = 3;

    public static double kLowerSVolts = 0.38834;
    public static double kLowerGVolts = 0.0942;
    public static double kLowerVVoltSecPerRad = 2.0427;
    public static double kLowerVVoltSecPerRadSquared = 0.23556;   
  }

  public static class LightConstants {
    public static final int kLightBarPWMPort = 9;
    public static final int kLightBarLength = 8;
  }

  public static class GregtechConstants {
    private static boolean loveForGreg = true; //always
    private static String thoughtsOnGreg = "Amazing and wonderful!";
    private static int numberOfHoursOnGreg = Integer.MAX_VALUE;
    private static int numberOfGregificationsCompleted = 2;
  }
}
