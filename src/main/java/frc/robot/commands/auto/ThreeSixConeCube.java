package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.util.Elements.Origin;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class ThreeSixConeCube extends AutoCommand {

    private PathPlannerTrajectory back;
    private PathPlannerTrajectory finish;

    public Trajectory getStartPosition() {
    if (DriverStation.getAlliance().equals(Alliance.Red)) {
        return mirrorPath(back);
    //    return back.transformBy(
    //     new Transform2d( //FieldConstants.kFieldLength - (back.getInitialPose().getX() * 2)
    //         new Translation2d(0, (back.getInitialPose().getY() * 2) - FieldConstants.kFieldWidth),
    //         new Rotation2d(Units.degreesToRadians(0))
    //     )
    //    );
    } else {
        return back;
    }
    }

    public static Trajectory mirrorPath(Trajectory traj) {
        var firstState = traj.getStates().get(0);
        var firstPose = traj.getInitialPose();

        // Calculate the transformed first pose.
        var newFirstPose = firstPose.plus(
            new Transform2d( //FieldConstants.kFieldLength - (back.getInitialPose().getX() * 2)
                new Translation2d(FieldConstants.kFieldLength - (firstPose.getX() * 2), 
                new Rotation2d(Units.degreesToRadians(180))),
                new Rotation2d(Units.degreesToRadians(0))
        ));
        List<State> newStates = new ArrayList<>();

        newStates.add(
            new State(
                firstState.timeSeconds,
                firstState.velocityMetersPerSecond,
                firstState.accelerationMetersPerSecondSq,
                newFirstPose,
                firstState.curvatureRadPerMeter));

        for (int i = 1; i < traj.getStates().size(); i++) {
        var state = traj.getStates().get(i);
        // We are transforming relative to the coordinate frame of the new initial pose.
        
        newStates.add(
            new State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                state.poseMeters.plus(new Transform2d(
                    new Translation2d(FieldConstants.kFieldLength - (state.poseMeters.getX() * 2), (state.poseMeters.getY() * 2) - FieldConstants.kFieldWidth),
                    new Rotation2d(180)
                )),
                state.curvatureRadPerMeter));
        }

        return new Trajectory(newStates);
    }

    public ThreeSixConeCube(Arm arm, Claw claw, Drivetrain drivetrain, RamseteAutoBuilder autoBuilder) {

        back = PathPlanner.loadPath("TestPath", new PathConstraints(1.75,5));

        finish = PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath("DriveUpToCubeGridWithCube", new PathConstraints(2,5)),
            DriverStation.getAlliance()
            );
        
        // drivetrain.resetPose(getStartPosition());

        this.addCommands(
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
}
