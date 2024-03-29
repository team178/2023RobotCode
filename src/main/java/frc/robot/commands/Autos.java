// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.BumpConeCube;
import frc.robot.commands.auto.BumpConeLeave;
import frc.robot.commands.auto.MidCubeCharge;
import frc.robot.commands.auto.PlaceConeAuto;
import frc.robot.commands.auto.SubConeCube;
import frc.robot.commands.auto.SubConeLeave;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public final class Autos {

    public static SendableChooser<AutoCommand> autoChooser = new SendableChooser<AutoCommand>();

    public static Command driveTrajectory(Drivetrain drive, Trajectory trajectory) {
        return new RamseteCommand(
            trajectory,
            drive::getEstimatedPosition,
            new RamseteController(),
            new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA),
            DriveConstants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.kPVel, 0, 0),
            new PIDController(DriveConstants.kPVel, 0, 0),
            drive::tankDriveVolts,
            drive
        ).andThen(() -> drive.tankDriveVolts(0, 0));
    }
    
    /*
    * Mirrors the provided trajectory across the field
    */
    public static Trajectory mirrorTrajectory(Trajectory traj) {

        // Calculate the transformed first pose.
        List<State> newStates = new ArrayList<>();

        for (var state : traj.getStates()) {
            newStates.add(
                new State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(
                        FieldConstants.kFieldLength - state.poseMeters.getX(),
                        state.poseMeters.getY(),
                        state.poseMeters.getRotation().times(-1).plus(
                        new Rotation2d(Units.degreesToRadians(180))
                        )
                    ),
            state.curvatureRadPerMeter));
        }

        return new Trajectory(newStates);
    }
    
    public static Trajectory chooseTrajectoryUsingAlliance(Trajectory blue, Trajectory red) {
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            return red;
        } else {
            return blue;
        }
    }
    
    public static void initAutoChooser(Arm arm, Claw claw, Drivetrain drivetrain) {
        autoChooser.setDefaultOption("None", new AutoCommand());

        autoChooser.setDefaultOption("PlaceHighOnly", new PlaceConeAuto(arm, claw));

        autoChooser.addOption("SubConeLeave", new SubConeLeave(arm, claw, drivetrain));
        autoChooser.addOption("SubConeCube", new SubConeCube(arm, claw, drivetrain));
        // autoChooser.addOption("SubConeCubeCharge", new SubConeCubeCharge(arm, claw, drivetrain));

        autoChooser.addOption("MidCubeCharge",/*parker wuz here*/ new MidCubeCharge(arm, claw, drivetrain));
        // autoChooser.addOption("MidCubeChargeOverBack", new MidCubeChargeOverBack(arm, claw, drivetrain));

        autoChooser.addOption("BumpConeLeave", new BumpConeLeave(arm, claw, drivetrain));
        autoChooser.addOption("BumpConeCube", new BumpConeCube(arm, claw, drivetrain));

        Shuffleboard.getTab("Autos")
            .add("Auto", autoChooser)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withSize(9, 1);
    }

    public static AutoCommand getSelectedAuto() {
        return autoChooser.getSelected();
    }

  public static Command placeHigh(Arm arm, Claw claw) {
    return Commands.sequence(
        claw.close(),
        arm.setPosition(ArmPosition.HIGH),
        new WaitCommand(1.5),
        claw.open(),
        new WaitCommand(0.3),
        arm.setPosition(ArmPosition.HOME));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
