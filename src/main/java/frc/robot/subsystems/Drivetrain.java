// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private final SPI.Port sPort = SPI.Port.kOnboardCS0;
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(sPort);
    
  private WPI_TalonFX m_leftMotor = new WPI_TalonFX(DriveConstants.kL1MotorPort);
  private WPI_TalonFX m_leftFollower = new WPI_TalonFX(DriveConstants.kL2MotorPort);

  private WPI_TalonFX m_rightMotor = new WPI_TalonFX(DriveConstants.kR1MotorPort);
  private WPI_TalonFX m_rightFollower = new WPI_TalonFX(DriveConstants.kR2MotorPort);

  private DifferentialDrivePoseEstimator m_poseEstimator;

  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kPVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kPVel, 0, 0);
  
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  private final Field2d m_field = new Field2d();

  /* Creates a new Drivetrain. */
  public Drivetrain() {

    m_leftMotor.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    m_rightMotor.configFactoryDefault();
    m_rightfollower.configFactoryDefault();

    m_leftFollower.follow(m_leftMotor);
    m_rightfollower.follow(m_rightMotor);

    //! Will need to be adjusted until we're going the right direction
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightfollower.setInverted(InvertType.FollowMaster);

    m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_poseEstimator = new DifferentialDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      getLeftEncoderPositionMeters(),
      getRightEncoderPositionMeters(),
      new Pose2d()
    );
  }

  public void resetGyro() {
    m_gyro.reset();
  }
  
  public void resetEncoders() {
    m_leftMotor.setSelectedSensorPosition(0);
    m_rightMotor.setSelectedSensorPosition(0);
  }

  public double getGyroHeading() {
    return m_gyro.getAngle();
  }

  public Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  public double getLeftEncoderPosition() {
    return m_leftMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderPositionMeters() {
    return talonUnitsToMeters(m_leftMotor.getSelectedSensorPosition());
  }
  
  public double getRightEncoderPosition() {
    return m_rightMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderPositionMeters() {
    return talonUnitsToMeters(m_rightMotor.getSelectedSensorPosition());
  }

  public double getLeftEncoderVelocity() {
    return m_leftMotor.getSelectedSensorVelocity();
  }

  public double getLeftEncoderVelocityMeters() {
    return talonUnitsToMeters(m_leftMotor.getSelectedSensorVelocity());
  }
  
  public double getRightEncoderVelocity() {
    return m_rightMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocityMeters() {
    return talonUnitsToMeters(m_rightMotor.getSelectedSensorVelocity());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftEncoderVelocityMeters(),
      getRightEncoderVelocityMeters()
    );
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        getLeftEncoderPositionMeters(),
        getRightEncoderPositionMeters(),
        pose
      );
  }

  private double talonUnitsToMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / DriveConstants.kEncoderCPR;
    double wheelRotations = motorRotations / DriveConstants.kGearboxRatio;
    double positionMeters = wheelRotations * DriveConstants.kEncoderDistancePerRev;
    return positionMeters;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
  }

  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(getLeftEncoderVelocityMeters(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(getRightEncoderVelocityMeters(), speeds.rightMetersPerSecond);
    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }
  
  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rot, double deadzone) {
    return this.run(() -> {
      arcadeDrive(
          MathUtil.applyDeadband(forward.getAsDouble(), deadzone),
          MathUtil.applyDeadband(rot.getAsDouble(), deadzone)
          );
    }).repeatedly();
  }
  
  public void arcadeDrive(double forward, double rot) {
    var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(
      new ChassisSpeeds(forward, 0.0, rot)
    );
    setWheelSpeeds(wheelSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_poseEstimator.update(getGyroRotation(), getRightEncoderPositionMeters(), getRightEncoderPositionMeters());
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
