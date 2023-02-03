// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class LowerArm extends SubsystemBase {
  
  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kLowerMotorPort, MotorType.kBrushless);
  private DigitalInput m_home = new DigitalInput(ArmConstants.kLowerHomePort);
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_controller;

  public LowerArm() {

    m_motor.restoreFactoryDefaults();

    m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_controller = m_motor.getPIDController();

    m_controller.setP(0.1);
    m_controller.setI(0);
    m_controller.setD(0);
    m_controller.setIZone(0);
    m_controller.setFF(0);
    m_controller.setOutputRange(-1, 1);
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
    setPosition(0);
  }

  public CommandBase resetEncoderCommand() {
    return this.runOnce(
      () -> this.resetEncoder()
    );
  }

  public boolean isHome() {
    return m_home.get();
  }

  public void setPosition(double rotations) {
    m_controller.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
