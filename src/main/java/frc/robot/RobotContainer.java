// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Autos;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;
import frc.robot.utils.Combo;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    private final UsbCamera camera0;
    private final UsbCamera camera1;

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final CommandXboxController m_auxBox =
        new CommandXboxController(OperatorConstants.kAuxControllerPort);

    private final Field2d m_autoPreview = new Field2d();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        PathPlannerServer.startServer(5811);

        camera0 = CameraServer.startAutomaticCapture(0);
        camera1 = CameraServer.startAutomaticCapture(1);

        // Configure the trigger bindings
        configureBindings();
        
        Autos.initAutoChooser(m_arm, m_claw, m_drivetrain);
        
        setupShuffleboard();
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

        m_drivetrain.setDefaultCommand(
            m_drivetrain.arcadeDrive(m_driverController::getLeftY, m_driverController::getRightX, 0.2));

        // Cheezy drive makes the falcons overheat
        // Why? I don't know.
        // Don't use it if you want to drive for more than 5 minutes
        // m_drivetrain.setDefaultCommand(
        //     m_drivetrain.cheesyDrive(m_driverController::getLeftY, m_driverController::getRightX, m_driverController.rightTrigger())
        // );

        // Drivebase slowdown triggers
        // The logic works so we'll just leave it
        // new Trigger(m_arm::isLowerHome)
        // .whileFalse(
        //   Commands.run(() -> m_drivetrain.setSpeedMult(0.05))
        // );
        m_driverController.leftTrigger()
            .whileTrue(
                Commands.run(() -> m_drivetrain.setSlowMode(true)))
            .whileFalse(
                Commands.run(() -> m_drivetrain.setSlowMode(false)));
        m_driverController.rightTrigger()
            .whileTrue(
                Commands.run(() -> m_drivetrain.setFastMode(true)))
            .whileFalse(
                Commands.run(() -> m_drivetrain.setFastMode(false)));

        m_auxBox.b().onTrue(
            m_arm.setPosition(ArmPosition.HOME));

        m_auxBox.y().onTrue(
            m_arm.setPosition(ArmPosition.SUBSTATION));

        m_auxBox.a().onTrue(
            m_arm.setPosition(ArmPosition.LOW));

        m_auxBox.x().onTrue(
            m_arm.setPosition(ArmPosition.HIGH));

        m_auxBox.leftBumper().onTrue(m_claw.toggle());

        // "Jog" functionality
        m_auxBox.povUpLeft().whileTrue(m_arm.bumpLower(-0.01));
        m_auxBox.povDownLeft().whileTrue(m_arm.bumpLower(0.01));

        m_auxBox.povUpRight().whileTrue(m_arm.bumpUpper(-0.01));
        m_auxBox.povDownRight().whileTrue(m_arm.bumpUpper(0.01));

        // Lights
        m_auxBox.leftStick().onTrue(m_lights.runYellow());
        m_auxBox.rightStick().onTrue(m_lights.runPurple());
        m_auxBox.leftTrigger().onTrue(m_lights.runDefaultColor());
        m_auxBox.rightTrigger().
        whileTrue(
            Commands.run(() -> m_drivetrain.setPauseMainControl(true)))
        .whileFalse(
            Commands.run(() -> m_drivetrain.setPauseMainControl(false)));

        // new Combo(m_auxBox.getHID())
        //     .quarterCircleKick()
        //     .onTrue(Autos.placeHigh(m_arm, m_claw));

    }

    private void setupShuffleboard() {

        // Auto selector
        Shuffleboard.getTab("Autos")
            .add(m_autoPreview)
            .withSize(9, 3);

        // Pre-flight indicators
        // make sure it's all green so we don't have to buy more gearboxes or NEOs
        ShuffleboardTab preflightTab = Shuffleboard.getTab("Pre-flight");
        ShuffleboardTab cameraTab = Shuffleboard.getTab("Cameras");

        preflightTab.addBoolean("Upper Limit Switch", m_arm::isUpperHome)
            .withPosition(0, 0);
        preflightTab.addBoolean("Lower Limit Switch", m_arm::isLowerHome)
            .withPosition(0, 1);

        preflightTab.addDouble("Lower Encoder", m_arm::getLowerPosition)
            .withPosition(1, 1);
        preflightTab.addDouble("Upper Encoder", m_arm::getUpperPosition)
            .withPosition(1, 0);

        preflightTab.addDouble("Lower Home Pos", () -> ArmPosition.HOME.lower)
            .withPosition(2, 1);
        preflightTab.addDouble("Upper Home Pos", () -> ArmPosition.HOME.upper)
            .withPosition(2, 0);

        preflightTab.addDouble("Level", m_drivetrain::getLevelHeading)
            .withPosition(0, 2);

        var positionList = preflightTab.getLayout("Arm Positions", BuiltInLayouts.kList)
            .withPosition(3, 0)
            .withSize(2, 3);

        positionList.add("Home", m_arm.setPosition(ArmPosition.HOME));
        positionList.add("Mid", m_arm.setPosition(ArmPosition.LOW));
        positionList.add("High", m_arm.setPosition(ArmPosition.HIGH));
        positionList.add("Substation", m_arm.setPosition(ArmPosition.SUBSTATION));
        positionList.add("Back", m_arm.setPosition(ArmPosition.BACK));
        
        preflightTab.add("Claw", m_claw.toggle()).withPosition(5, 0);

        cameraTab.add("Camera", CameraServer.getVideo().getSource());
        
    }
    
    public void disabledPeriodic() {
        Pose2d selectedAutoPose = Autos.getSelectedAuto().getStartPosition();
        m_autoPreview.setRobotPose(selectedAutoPose);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        AutoCommand selectedAuto = Autos.getSelectedAuto();
        m_drivetrain.resetPose(selectedAuto.getStartPosition());
        return selectedAuto;
    }
}
