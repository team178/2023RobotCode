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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmPosition;

public class Arm extends SubsystemBase {
  
  private CANSparkMax m_lowerMotor = new CANSparkMax(ArmConstants.kLowerMotorPort, MotorType.kBrushless);
  private CANSparkMax m_upperMotor = new CANSparkMax(ArmConstants.kUpperMotorPort, MotorType.kBrushless);
  
  private DigitalInput m_lowerHome = new DigitalInput(ArmConstants.kLowerHomePort);
  private DigitalInput m_upperHome = new DigitalInput(ArmConstants.kUpperHomePort);

  private DutyCycleEncoder m_lowerEncoder = new DutyCycleEncoder(ArmConstants.kLowerArmEncoder);
  private DutyCycleEncoder m_upperEncoder = new DutyCycleEncoder(ArmConstants.kUpperArmEncoder);

  private PIDController m_lowerController = new PIDController(ArmConstants.kLowerArmP, 0, 0);
  private PIDController m_upperController = new PIDController(ArmConstants.kUpperArmP, 0, 0);

  private ArmFeedforward m_lowerFeedforward =
        new ArmFeedforward(
            ArmConstants.kLowerSVolts, ArmConstants.kLowerGVolts,
            ArmConstants.kLowerVVoltSecPerRad, ArmConstants.kLowerVVoltSecPerRadSquared
        );

  private ArmFeedforward m_upperFeedforward =
        new ArmFeedforward(
            ArmConstants.kUpperSVolts, ArmConstants.kUpperGVolts,
            ArmConstants.kUpperVVoltSecPerRad, ArmConstants.kUpperVVoltSecPerRadSquared
        );
  
  private MechanismLigament2d m_lowerArm2d;
  private MechanismLigament2d m_upperArm2d;

  private ArmPosition m_position = ArmPosition.HOME;

   public Arm() {
    m_lowerMotor.restoreFactoryDefaults();
    m_upperMotor.restoreFactoryDefaults();

    m_lowerMotor.setIdleMode(IdleMode.kBrake);
    m_upperMotor.setIdleMode(IdleMode.kBrake);

    m_lowerMotor.setInverted(true);
    m_upperMotor.setInverted(true);

    m_lowerEncoder.setDistancePerRotation(2 * Math.PI);
    m_upperEncoder.setDistancePerRotation(2 * Math.PI);

    setBrake();

    Mechanism2d armMech = new Mechanism2d(100, 60);
    MechanismRoot2d armRoot = armMech.getRoot("armPivot", 60, 15);
    armRoot.append(new MechanismLigament2d("Pylon", 15, -90, 6, new Color8Bit(0, 0, 255)));
    m_lowerArm2d = armRoot.append(
        new MechanismLigament2d("LowerArm", 28, -13));
    m_upperArm2d = m_lowerArm2d.append(
        new MechanismLigament2d("UpperArm", 28, 90));

    SmartDashboard.putData("ArmMech2d", armMech);

    setLowerPosition(m_position.lower);
    setUpperPosition(m_position.upper);    

    // m_lowerEncoder.setPositionOffset(Units.degreesToRotations(-8) - (7/6));
    // m_upperEncoder.setPositionOffset(Units.degreesToRotations(-14));
  }

  public void setBrake() {
    m_lowerMotor.setIdleMode(IdleMode.kBrake);
    m_upperMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
      m_lowerMotor.setIdleMode(IdleMode.kCoast);
      m_upperMotor.setIdleMode(IdleMode.kCoast);
  }
  
  public void resetLowerEncoder() {
    m_lowerEncoder.reset();
  }

  public void resetUpperEncoder() {
    m_upperEncoder.reset();
  }

  public void resetEncoders() {
    resetLowerEncoder();
    resetUpperEncoder();
  }
  
  public CommandBase resetEncoderCommand() {
    return this.runOnce(
        () -> this.resetEncoders());
  }

  public CommandBase setPosition(ArmPosition pos) {
    return this.runOnce(
      () -> {
          m_position = pos;
          if (!pos.equals(ArmPosition.HOLD)) {
            setLowerPosition(pos.lower);
            setUpperPosition(pos.upper);       
          }
      }
    );
  }
  
  public void setLowerPosition(double pos) {
    m_lowerController.setSetpoint(pos);
  }

  public void setUpperPosition(double pos) {
    m_upperController.setSetpoint(pos);
  }

  public void bumpLowerPosition(double bump) {
    setLowerPosition(m_lowerController.getSetpoint() + bump);
  }

  public void bumpUpperPosition(double bump) {
    setUpperPosition(m_upperController.getSetpoint() + bump);
  }

  public CommandBase bumpLower(double bump) {
    return Commands.run(() -> {
      bumpLowerPosition(bump);
    });
  }

  public CommandBase bumpUpper(double bump) {
    return Commands.run(() -> {
      bumpUpperPosition(bump);
    });
  }

  public boolean isLowerHome() {
    return m_lowerHome.get();
  }

  public boolean isUpperHome() {
    return m_upperHome.get();
  }

  public double getLowerPosition() {
    return Units.rotationsToRadians(m_lowerEncoder.getAbsolutePosition() - m_lowerEncoder.getPositionOffset() - (1/6));
  }

  public double getUpperPosition() {
    return Units.rotationsToRadians(m_upperEncoder.getAbsolutePosition() - m_upperEncoder.getPositionOffset());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (isLowerHome()) {
      // resetLowerEncoder();
      // m_lowerEncoder.setPositionOffset(Units.degreesToRotations(-8) - (7/6));
    // }

    // if (isUpperHome()) {
    //  resetUpperEncoder();
      // m_upperEncoder.setPositionOffset(Units.degreesToRotations(-14));
    // }

    // if (m_position.equals(ArmPosition.HOLD)) {
    //   setLowerPosition(getLowerPosition());
    //   setUpperPosition(getUpperPosition());
    // }

    double lowerFF = m_lowerFeedforward.calculate(m_lowerController.getSetpoint(), 0);
    double upperFF = m_upperFeedforward.calculate(m_upperController.getSetpoint(), 0);

    double lowerOut = -m_lowerController.calculate(getLowerPosition());
    double upperOutput = -m_upperController.calculate(getUpperPosition());

    // This is all the limit switches do :)
    
    if (lowerOut < 0.1 && isLowerHome()) {
      m_lowerMotor.setVoltage(0);
    } else {
      m_lowerMotor.setVoltage(lowerOut + lowerFF);
    }

    if (upperOutput < 0.1 && isUpperHome()) {
      m_upperMotor.setVoltage(0);
    } else {
      m_upperMotor.setVoltage(upperOutput + upperFF);
    }

    SmartDashboard.putNumber("LowerPosition", getLowerPosition());
    SmartDashboard.putNumber("LowerSetpoint", m_lowerController.getSetpoint());
    SmartDashboard.putNumber("LowerOutput", lowerOut);
    SmartDashboard.putNumber("LowerOutputFF", lowerOut + lowerFF);
    
    SmartDashboard.putNumber("UpperPosition", getUpperPosition());
    SmartDashboard.putNumber("UpperSetpoint", m_upperController.getSetpoint());
    SmartDashboard.putNumber("UpperOutput", upperOutput);
    SmartDashboard.putNumber("UpperOutputFF", upperOutput + upperFF);

    m_lowerArm2d.setAngle(180 + Units.radiansToDegrees(getLowerPosition()));
    m_upperArm2d.setAngle(180 - Units.radiansToDegrees(getUpperPosition()));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
