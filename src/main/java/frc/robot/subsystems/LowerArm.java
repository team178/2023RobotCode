// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class LowerArm extends ProfiledPIDSubsystem {
  
  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kLowerMotorPort, MotorType.kBrushless);
  private DigitalInput m_home = new DigitalInput(ArmConstants.kLowerHomePort);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kLowerArmEncoder);

  private ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kLowerSVolts, ArmConstants.kLowerGVolts,
          ArmConstants.kLowerVVoltSecPerRad, ArmConstants.kLowerVVoltSecPerRadSquared
      );

  public LowerArm() {
    super(
      new ProfiledPIDController(
        ArmConstants.kLowerArmP,
        0,
        0,
        new TrapezoidProfile.Constraints(
          ArmConstants.kLowerMaxRadsPerSec,
          ArmConstants.kLowerMaxRadsPerSecSquared
        )
      ),
        0);
    
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_encoder.setDistancePerRotation(2 * Math.PI);
    resetEncoder();
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
    m_encoder.reset();
  }
  
  public CommandBase resetEncoderCommand() {
    return this.runOnce(
        () -> this.resetEncoder());
  }
  
  public CommandBase setGoalCommand(double goal) {
    return this.runOnce(
        () -> this.setGoal(goal)
    );
  }

  public boolean isHome() {
    return m_home.get();
  }

  public double getPosition() {
    return m_encoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isHome()) {
      resetEncoder();
      m_encoder.setPositionOffset(Units.degreesToRadians(-8));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    m_motor.setVoltage(output + feedforward);
    
  }

  @Override
  protected double getMeasurement() {
    return getPosition() + ArmConstants.kLowerOffsetRads;
  }
}
