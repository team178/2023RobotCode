// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class UpperArm extends SubsystemBase {
  
  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kUpperMotorPort, MotorType.kBrushless);
  private DigitalInput m_home = new DigitalInput(ArmConstants.kUpperHomePort);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kUpperArmEncoder);

  public UpperArm() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  public void setBrake() {
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    m_motor.setIdleMode(IdleMode.kCoast);
  }
  
  /*
   * Offset encoder back to zero
   */
  public void resetEncoder() {
    m_encoder.setPositionOffset(getPosition());
  }
  
  public CommandBase resetEncoderCommand() {
    return this.runOnce(
      () -> this.resetEncoder()
    );
  }

  public boolean isHome() {
    return m_home.get();
  }

  public double getPosition() {
    return m_encoder.getAbsolutePosition();
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
