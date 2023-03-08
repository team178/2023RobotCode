// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public final class Autos {

  public static final HashMap<String, Command> eventMap = initEventMap();

  private static HashMap<String, Command> initEventMap() {
    HashMap<String, Command> events = new HashMap<>();
    // Add events that can be used in a Pathplanner path here
    // events.put("ExampleMarker", new ExampleCommand());
    return events;
  }

  // public static Command driveTrajectory(Drivetrain drive, Trajectory trajectory) {
  //   return new RamseteCommand(
  //       trajectory,
  //       drive::getEstimatedPosition,
  //       new RamseteController(),
  //       new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA),
  //       DriveConstants.kDriveKinematics,
  //       drive::getWheelSpeeds,
  //       new PIDController(DriveConstants.kPVel, 0, 0),
  //       new PIDController(DriveConstants.kPVel, 0, 0),
  //       drive::tankDriveVolts,
  //       drive
  //     ).andThen(() -> drive.tankDriveVolts(0, 0));
  // }

  public static Command blueSixConeCube(Arm arm, Claw claw, Drivetrain drivetrain, RamseteAutoBuilder autoBuilder) {
    PathPlannerTrajectory back = PathPlanner.loadPath("TestPath", new PathConstraints(1.75,5));
    PathPlannerTrajectory finish = PathPlanner.loadPath("DriveUpToCubeGridWithCube", new PathConstraints(2,5));
    drivetrain.resetPose(
      new Pose2d(1.83, 4.94, new Rotation2d(Units.degreesToRadians(180)))
    );
    return Commands.sequence(
      Autos.placeHigh(arm, claw),
      new WaitCommand(0.2),
      Commands.parallel(
        autoBuilder.followPath(back),
        Commands.sequence(
          new WaitCommand(0.7),
          arm.setPosition(ArmPosition.BACK),
          claw.open()
          )
      ),
      claw.open(),
      new WaitCommand(0.3),
      arm.setPosition(ArmPosition.HOME),
      autoBuilder.followPath(finish),
      Autos.placeHigh(arm, claw),
      new WaitCommand(0.2)
    );
  }

  public static Command placeHigh(Arm arm, Claw claw) {
    return Commands.sequence(
      claw.close(),
      arm.setPosition(ArmPosition.HIGH),
      new WaitCommand(1.5),
      claw.open(),
      new WaitCommand(0.3),
      arm.setPosition(ArmPosition.HOME)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
