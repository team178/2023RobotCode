// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class UpperArm extends SubsystemBase {
  
  private CANSparkMax m_motor = new CANSparkMax(ArmConstants.kUpperMotorPort, MotorType.kBrushless);
  private DigitalInput m_home = new DigitalInput(ArmConstants.kUpperHomePort);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kUpperArmEncoder);

  private PIDController m_controller = new PIDController(ArmConstants.kUpperArmP, 0, 0);

  private ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kUpperSVolts, ArmConstants.kUpperGVolts,
          ArmConstants.kUpperVVoltSecPerRad, ArmConstants.kUpperVVoltSecPerRadSquared
      );

  public UpperArm() {
    
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);
    m_encoder.setDistancePerRotation(2 * Math.PI);
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
    // setGoal(m_encoder.getDistance());
  }
  
  public CommandBase resetEncoderCommand() {
    return this.runOnce(
        () -> this.resetEncoder());
  }

  public void setGoal(double goal) {
    m_controller.setSetpoint(goal);
  }
  
  public CommandBase setGoalCommand(double goal) {
    return this.runOnce(
        () -> m_controller.setSetpoint(goal)
    );
  }

  public void bumpGoal(double bump) {
    setGoal(m_controller.getSetpoint() + bump);
  }

  public CommandBase bumpGoalCommand(double bump) {
    return this.run(
        () -> bumpGoal(bump)
    );
  }

  public boolean isHome() {
    return !m_home.get();
  }

  public double getPosition() {
    return m_encoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isHome()) {
      resetEncoder();
      //! somehow this works but it's not how it's supposed to work but it works so don't ask questions
      m_encoder.setPositionOffset(Units.degreesToRadians(-14));
    }

    double feedforward = m_feedforward.calculate(m_controller.getSetpoint(), 0);

    double output = -m_controller.calculate(getPosition());

    SmartDashboard.putNumber("Setpoint Upper", m_controller.getSetpoint());
    SmartDashboard.putNumber("OUTPUT_Upper", output);
    SmartDashboard.putNumber("OUTPUT_Upper_FF", output + feedforward);

    if (output < 0.1 && isHome()) {
      m_motor.setVoltage(0);
    } else {
      m_motor.setVoltage(output + feedforward);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
