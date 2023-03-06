// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Lights m_lights = new Lights();

  private final Arm m_arm = new Arm();
  private final Claw m_claw = new Claw();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_auxBox =
      new CommandXboxController(OperatorConstants.kAuxControllerPort);
    
  private final RamseteAutoBuilder m_autoBuilder = new RamseteAutoBuilder(
    m_drivetrain::getEstimatedPosition,
    m_drivetrain::resetPose,
    new RamseteController(),
    DriveConstants.kDriveKinematics,
    m_drivetrain::tankDriveVolts,
    Autos.eventMap,
    m_drivetrain
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    PathPlannerServer.startServer(5811);

    CameraServer.startAutomaticCapture();

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_drivetrain.setDefaultCommand(
        m_drivetrain.arcadeDrive(m_driverController::getLeftY, m_driverController::getRightX, 0.2)
    );

    // m_driverController.leftTrigger().whileTrue(
    //   Commands.run(() -> m_drivetrain.setSpeedMult(0.5))
    // );

    new Trigger(m_arm::isLowerHome)
      .whileTrue(
        Commands.runOnce(() -> m_drivetrain.setSpeedMult(1))
      )
      .whileFalse(
        Commands.run(() -> m_drivetrain.setSpeedMult(0.5))
    );

    m_auxBox.b().onTrue(
      m_arm.setPosition(ArmPosition.HOME)
    );
    
    m_auxBox.y().onTrue(
      m_arm.setPosition(ArmPosition.SUBSTATION)
    );

    m_auxBox.a().onTrue(
      m_arm.setPosition(ArmPosition.LOW)
    );

    m_auxBox.x().onTrue(
      m_arm.setPosition(ArmPosition.HIGH)
    );

    m_auxBox.rightTrigger().onTrue(
      m_arm.setPosition(ArmPosition.BACK)
    );

    m_auxBox.leftBumper().onTrue(
      Commands.runOnce(() -> {
        m_claw.toggle();
      })
    );

    m_auxBox.povUpLeft().whileTrue(m_arm.bumpLower(-0.01));
    m_auxBox.povDownLeft().whileTrue(m_arm.bumpLower(0.01));

    m_auxBox.povUpRight().whileTrue(m_arm.bumpUpper(-0.01));
    m_auxBox.povDownRight().whileTrue(m_arm.bumpUpper(0.01));

    m_auxBox.leftStick().onTrue(m_lights.runYellow());
    m_auxBox.rightStick().onTrue(m_lights.runPurple());
    m_auxBox.leftTrigger().onTrue(m_lights.runDefaultColor());
  }

  public void periodic() {
    SmartDashboard.putNumber("Ultrasonic", m_claw.getUltrasonicDistance());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TestPath", new PathConstraints(3,1));
    return m_autoBuilder.followPathGroup(pathGroup);
  }
}
