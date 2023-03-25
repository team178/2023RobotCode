// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.CheesyDriveHelper;
import frc.robot.utils.DriveSignal;
import frc.robot.utils.LimelightHelpers;

public class Drivetrain extends SubsystemBase {

  private final SPI.Port sPort = SPI.Port.kOnboardCS0;
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(sPort);
  private final AnalogGyro m_level = new AnalogGyro(0);

  private WPI_TalonFX m_leftMotor = new WPI_TalonFX(DriveConstants.kL1MotorPort);
  private WPI_TalonFX m_leftFollower = new WPI_TalonFX(DriveConstants.kL2MotorPort);

  private WPI_TalonFX m_rightMotor = new WPI_TalonFX(DriveConstants.kR1MotorPort);
  private WPI_TalonFX m_rightFollower = new WPI_TalonFX(DriveConstants.kR2MotorPort);

  private DifferentialDrivePoseEstimator m_poseEstimator;

  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kPVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kPVel, 0, 0);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV,
      DriveConstants.kA);

  private final CheesyDriveHelper m_cheesyHelper = new CheesyDriveHelper();

  private final Field2d m_field = new Field2d();
  private final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private double m_speedMult = 1;

  /* Creates a new Drivetrain. */
  public Drivetrain() {

    m_leftMotor.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    m_rightMotor.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);

    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);

    m_leftFollower.follow(m_leftMotor);
    m_rightFollower.follow(m_rightMotor);

    m_leftFollower.setSensorPhase(false);
    m_rightFollower.setSensorPhase(false);

    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Current limiting to prevent brownouts
    StatorCurrentLimitConfiguration cur_limit = new StatorCurrentLimitConfiguration(true, 80, 70, 0.5);
    m_leftMotor.configStatorCurrentLimit(cur_limit);
    m_leftFollower.configStatorCurrentLimit(cur_limit);
    m_rightMotor.configStatorCurrentLimit(cur_limit);
    m_rightFollower.configStatorCurrentLimit(cur_limit);

    calibrateGyro();
    resetEncoders();

    m_poseEstimator = new DifferentialDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroRotation(),
        getLeftEncoderPositionMeters(),
        getRightEncoderPositionMeters(),
        new Pose2d());

    // Weird matrix thing that we can use to adjust how much we trust each of the Limelight's input values
    m_poseEstimator.setVisionMeasurementStdDevs(DriveConstants.kVisionTrustMatrix);
  }

  public void resetGyro() {
    m_gyro.reset();
    m_level.reset();
  }

  public void calibrateGyro() {
    m_gyro.calibrate();
    m_level.calibrate();
  }

  public void resetEncoders() {
    m_leftMotor.setSelectedSensorPosition(0);
    m_rightMotor.setSelectedSensorPosition(0);
  }

  /*
   * Multiplier (technically a divider but whatever) for the drivetrain speed
   * `speed` should be a number between 0.0 and 1.0, which is multiplied by the
   * max speed in meters per second.
   * Just meant to be a quick and easy speed multiplier.
   */
  public void setSpeedMult(double speed) {
    m_speedMult = MathUtil.clamp(speed, 0.0, 1.0);
  }

  public double getGyroHeading() {
    return m_gyro.getAngle();
  }

  public double getLevelHeading() {
    return m_level.getAngle();
  }

  public Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  public double getLeftEncoderPosition() {
    return m_leftMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderPositionMeters() {
    return talonUnitsToMeters(m_leftMotor.getSelectedSensorPosition() / DriveConstants.kEncoderCPR);
  }

  public double getRightEncoderPosition() {
    return m_rightMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderPositionMeters() {
    return talonUnitsToMeters(m_rightMotor.getSelectedSensorPosition() / DriveConstants.kEncoderCPR);
  }

  public double getLeftEncoderVelocity() {
    return m_leftMotor.getSelectedSensorVelocity();
  }

  public double getLeftEncoderVelocityMeters() {
    return talonUnitsToMeters(m_leftMotor.getSelectedSensorVelocity() / DriveConstants.kEncoderCPR * 10);
  }

  public double getRightEncoderVelocity() {
    return m_rightMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocityMeters() {
    return talonUnitsToMeters(m_rightMotor.getSelectedSensorVelocity() / DriveConstants.kEncoderCPR * 10);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftEncoderVelocityMeters(),
        getRightEncoderVelocityMeters());
  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        getLeftEncoderPositionMeters(),
        getRightEncoderPositionMeters(),
        pose);
  }

  private double talonUnitsToMeters(double motorRotations) {
    double wheelRotations = motorRotations * DriveConstants.kGearboxRatio;
    double positionMeters = wheelRotations * DriveConstants.kEncoderDistancePerRev;
    return positionMeters;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
  }

  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setWheelSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
  }

  public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
    final double leftFeedforward = m_feedforward.calculate(leftSpeed);
    final double rightFeedforward = m_feedforward.calculate(rightSpeed);

    final double leftOutput = m_leftPIDController.calculate(getLeftEncoderVelocityMeters(), leftSpeed);
    final double rightOutput = m_rightPIDController.calculate(getRightEncoderVelocityMeters(), rightSpeed);
    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rot, double deadzone) {
    return this.run(() -> {
      double x = MathUtil.applyDeadband(forward.getAsDouble(), deadzone)
          * (DriveConstants.kMaxSpeedMetersPerSecond * m_speedMult);
      double z = MathUtil.applyDeadband(rot.getAsDouble(), deadzone)
          * (DriveConstants.kMaxRotationSpeedMetersPerSecond * m_speedMult);
      arcadeDrive(
          x,
          z);
    }).repeatedly();
  }

  public void arcadeDrive(double forward, double rot) {
    var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(
        new ChassisSpeeds(-forward, 0.0, -rot));
    setWheelSpeeds(wheelSpeeds);
  }
  
  public Command cheesyDrive(DoubleSupplier forward, DoubleSupplier rot) {
    return this.run(() -> {

      double throttle = forward.getAsDouble();

      DriveSignal signal = m_cheesyHelper.cheesyDrive(
          -throttle,
          rot.getAsDouble() * 0.8,
          Math.abs(throttle) < 0.3, // Quick turn range, for all I know this might be better on a trigger
          false
      );

      setWheelSpeeds(
        signal.getLeft() * DriveConstants.kMaxSpeedMetersPerSecond * m_speedMult,
        signal.getRight() * DriveConstants.kMaxSpeedMetersPerSecond * m_speedMult
      );

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_poseEstimator.update(getGyroRotation(), getRightEncoderPositionMeters(), getRightEncoderPositionMeters());

    if (m_limelight.containsKey("botpose")) {

      double[] botposeEntry = LimelightHelpers.getBotPose("limelight");

      if (LimelightHelpers.getTV("limelight")) {
        // The pose from limelight for some reason has it's orign in the middle of the
        // field instead
        // of the bottom left like the WPILib pose estimator, so we have to account for
        // that%
        Pose2d botpose = new Pose2d(
            botposeEntry[0] + FieldConstants.kFieldLength / 2,
            botposeEntry[1] + FieldConstants.kFieldWidth / 2,
            Rotation2d.fromDegrees(botposeEntry[5]));

        m_poseEstimator.addVisionMeasurement(
            botpose,
            Timer.getFPGATimestamp() - (botposeEntry[6]/1000));
      }
    }

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putData(m_field);
    SmartDashboard.putData(m_gyro);

    // SmartDashboard.putNumber("Left Temp.", m_leftMotor.getTemperature());
    // SmartDashboard.putNumber("Right Temp.", m_rightMotor.getTemperature());

    // SmartDashboard.putNumber("Left aTemp.", m_leftFollower.getTemperature());
    // SmartDashboard.putNumber("Right aTemp.", m_rightFollower.getTemperature());

    // SmartDashboard.putNumber("LeftCurrent", m_leftMotor.getStatorCurrent());
    // SmartDashboard.putNumber("RightCurrent", m_rightMotor.getStatorCurrent());

    // SmartDashboard.putNumber("LeftVolts", m_leftMotor.getMotorOutputVoltage());
    // SmartDashboard.putNumber("RightVolts",m_leftMotor.getMotorOutputVoltage());

    // SmartDashboard.putNumber("LeftEncoderVel", getLeftEncoderVelocityMeters());
    // SmartDashboard.putNumber("RightEncoderVel", getRightEncoderVelocityMeters());

    // SmartDashboard.putNumber("LeftEncoderPos", getLeftEncoderPositionMeters());
    // SmartDashboard.putNumber("RightEncoderPos", getLeftEncoderPositionMeters());

    // SmartDashboard.putNumber("LeftSetpoint", m_rightPIDController.getSetpoint());
    // SmartDashboard.putNumber("RightSetpoint", m_rightPIDController.getSetpoint());
    // SmartDashboard.putNumber("level", getLevelHeading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
