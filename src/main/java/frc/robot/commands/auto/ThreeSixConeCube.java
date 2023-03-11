package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class ThreeSixConeCube extends AutoCommand {

    private Trajectory back;
    private Trajectory finish;

    @Override
    public Pose2d getStartPosition() {
        return back.getInitialPose();
    }

    public ThreeSixConeCube(Arm arm, Claw claw, Drivetrain drivetrain) {

        back = Autos.mirrorTrajectoryIfRed(
                PathPlanner.loadPath("TestPath", new PathConstraints(1.75, 5))
            );

        finish = Autos.mirrorTrajectoryIfRed(
                PathPlanner.loadPath("DriveUpToCubeGridWithCube", new PathConstraints(2,5))
            );

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2),
            Commands.parallel(
                Autos.driveTrajectory(drivetrain, back),
                Commands.sequence(
                    new WaitCommand(0.7),
                    arm.setPosition(ArmPosition.BACK),
                    claw.open()
                )
            ),
            claw.open(),
            new WaitCommand(0.3),
            arm.setPosition(ArmPosition.HOME),
            Autos.driveTrajectory(drivetrain, finish),
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2)
        );
    }
}
