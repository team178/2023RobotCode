// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayTopic;

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

    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxRotationSpeedMetersPerSecond = 3.0;

    public static final double kS = 1; //! Need to be tuned
    public static final double kV = 3;
    public static final double kA = 1;

    public static final double kPVel = 1;

    public static final Matrix<N3, N1> kVisionTrustMatrix = m_createVisionTrustMatrix();

    private static Matrix<N3, N1> m_createVisionTrustMatrix() {
      Matrix<N3, N1> matrix = new Matrix<N3, N1>(N3.instance, N1.instance);
      matrix.set(0, 0, 3); // X
      matrix.set(1, 0, 3); // Y
      matrix.set(2, 0, 3); // Theta
      return matrix;
    }
  }

  public static class ClawConstants {
    public static final int kFwdChannel = 0;
    public static final int kRevChannel = 1;
  }
  
  public static class ArmConstants {
    public static final int kLowerMotorPort = 20;
    public static final int kUpperMotorPort = 21;

    public static final int kLowerHomePort = 0;
    public static final int kUpperHomePort = 1;

    public static final int kLowerArmEncoder = 2;
    public static final int kUpperArmEncoder = 3;
    
    // Upper arm
    public static double kUpperOffsetRads = 0;
    public static double kUpperMaxRadsPerSec = 0.1;
    public static double kUpperMaxRadsPerSecSquared = 0.1;

    public static double kUpperArmP = 0;    
    public static double kUpperSVolts = 0;
    public static double kUpperGVolts = 0;
    public static double kUpperVVoltSecPerRad = 0;
    public static double kUpperVVoltSecPerRadSquared = 0;
    
    // Lower arm
    public static double kLowerOffsetRads = 0;
    public static double kLowerMaxRadsPerSec = 0.1;
    public static double kLowerMaxRadsPerSecSquared = 0.1;

    public static double kLowerSVolts = 0;
    public static double kLowerGVolts = 0;
    public static double kLowerVVoltSecPerRad = 0;
    public static double kLowerVVoltSecPerRadSquared = 0;

    public static double kLowerArmP = 0;    
  }
}
