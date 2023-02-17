// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.Pulse;
import frc.robot.subsystems.UpperArm;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
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
  public final Pulse m_pulse = new Pulse();
  
  private final UpperArm m_upperArm = new UpperArm();
  private final LowerArm m_lowerArm = new LowerArm();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
  private final RamseteAutoBuilder m_autoBuilder = new RamseteAutoBuilder(
    m_drivetrain::getEstimatedPosition,
    m_drivetrain::resetPose,
    new RamseteController(),
    DriveConstants.kDriveKinematics,
    m_drivetrain::tankDriveVolts,
    Autos.eventMap,
    m_drivetrain
  );

  private MechanismLigament2d m_lowerArm2d;
  private MechanismLigament2d m_upperArm2d;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    PathPlannerServer.startServer(5811);

    // Configure the trigger bindings
    configureBindings();

    Mechanism2d armMech = new Mechanism2d(100, 60);
    MechanismRoot2d armRoot = armMech.getRoot("armPivot", 60, 15);
    armRoot.append(new MechanismLigament2d("Pylon", 15, -90, 6, new Color8Bit(0, 0, 255)));
    m_lowerArm2d = armRoot.append(
        new MechanismLigament2d("LowerArm", 28, -13));
    m_upperArm2d = m_lowerArm2d.append(
        new MechanismLigament2d("UpperArm", 28, 90));

    SmartDashboard.putData("ArmMech2d", armMech);
  }
  
  public void updateMech2d() {
    m_lowerArm2d.setAngle(180 + Units.radiansToDegrees(m_lowerArm.getPosition()));
    m_upperArm2d.setAngle(180 - Units.radiansToDegrees(m_upperArm.getPosition()));
    SmartDashboard.putNumber("lower", m_lowerArm.getPosition());
    SmartDashboard.putNumber("upper", m_upperArm.getPosition());
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
        m_drivetrain.arcadeDrive(m_driverController::getLeftY, m_driverController::getLeftY, 0.2)
    );
    // m_lowerArm.enable();

    m_lowerArm.setGoal(0.153306);
    m_upperArm.setGoal(5.999611);

    m_driverController.a().onTrue(
      Commands.run(() -> {
        m_lowerArm.setGoal(-1.979749);
        m_upperArm.setGoal(3.787933);
        System.out.println("a");
      })
    );

    m_driverController.b().onTrue(
      Commands.run(() -> {
        m_lowerArm.setGoal(0.153306);
        m_upperArm.setGoal(5.999611);
        System.out.println("b");
      })
    );

    m_lowerArm.setBrake();
    m_upperArm.setBrake();

    m_driverController.y().onTrue(m_pulse.runLightsOff());
    m_driverController.x().onTrue(m_pulse.runRedBlue());
    m_driverController.x().onFalse(m_pulse.runLightsOff());
  }

  public void onDisable() {
    // m_lowerArm.setCoast();
    // m_upperArm.setCoast();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TestPath", new PathConstraints(1, 3));
    return m_autoBuilder.fullAuto(pathGroup);
  }
}
