// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to
 * follow a trajectory
 * {@link Trajectory} with a differential drive.
 */
public class DriveTrajectory extends CommandBase {
    private final Timer m_timer = new Timer();
    private Supplier<Trajectory> m_trajectorySupplier;
    private Trajectory m_trajectory;
    
    private final RamseteController m_follower = new RamseteController();
    private final DifferentialDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;
    private final Supplier<Pose2d> m_pose;
    private final BiConsumer<Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    public DriveTrajectory(Drivetrain drive, Trajectory trajectory) {
        m_trajectory = trajectory;
        m_pose = drive::getEstimatedPosition;
        m_output = drive::setWheelSpeeds;
        addRequirements(drive);
    }

    public DriveTrajectory(Drivetrain drive, Supplier<Trajectory> trajectory) {
        m_trajectorySupplier = trajectory;
        m_pose = drive::getEstimatedPosition;
        m_output = drive::setWheelSpeeds;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (m_trajectorySupplier != null) {
            m_trajectory = m_trajectorySupplier.get();
        }
        m_prevTime = -1;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds =
            m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(
                    initialState.velocityMetersPerSecond,
                    0,
                    initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.restart();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();

        if (m_prevTime < 0) {
            m_output.accept(0.0, 0.0);
            m_prevTime = curTime;
            return;
        }

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        m_output.accept(leftSpeedSetpoint, rightSpeedSetpoint);
        m_prevSpeeds = targetWheelSpeeds;
        m_prevTime = curTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();

        if (interrupted) {
            m_output.accept(0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("leftVelocity", () -> m_prevSpeeds.leftMetersPerSecond, null);
        builder.addDoubleProperty("rightVelocity", () -> m_prevSpeeds.rightMetersPerSecond, null);
    }
}
