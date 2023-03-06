// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

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

  public static Command placeCone(Arm arm, Claw claw) {
    return Commands.sequence(
      Commands.runOnce(claw::close),
      arm.setPosition(ArmPosition.HIGH),
      new WaitCommand(2),
      Commands.runOnce(claw::open),
      new WaitCommand(1),
      arm.setPosition(ArmPosition.HOME),
      new WaitCommand(0.5),
      Commands.runOnce(claw::close)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
