package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class BumpConeCube extends AutoCommand {
    private AutoTrajectoryPair toCube;
    private AutoTrajectoryPair toGrid;

    @Override
    public Pose2d getStartPosition() {
        return toCube.getAllianceTrajectory().getInitialPose();
    }

    public BumpConeCube(Arm arm, Claw claw, Drivetrain drivetrain) {

        toCube = new AutoTrajectoryPair(PathPlanner.loadPath("BumpDriveToCube", new PathConstraints(1.0, 5), true));
        toGrid = new AutoTrajectoryPair(PathPlanner.loadPath("BumpDriveToGrid", new PathConstraints(1.0,5)));

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2),
            Commands.parallel(
                new DriveTrajectory(drivetrain, toCube::getAllianceTrajectory),
                Commands.sequence(
                    new WaitCommand(2),
                    arm.setPosition(ArmPosition.BACK),
                    claw.open()
                )
            ).deadlineWith(
                Commands.waitSeconds(0.7) // delay is here so that the photosensor does not trigger before it gets out of range of the cone
                    .andThen(
                        Commands.waitUntil(claw::getPhotosensor).andThen(claw.close()))
            ),
            claw.close(),
            new WaitCommand(0.3),
            arm.setPosition(ArmPosition.HOME),
            new DriveTrajectory(drivetrain, toGrid::getAllianceTrajectory),
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2)
        );
    }
}
