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

public class SubConeCubeCharge extends AutoCommand {

    private AutoTrajectoryPair toCube;
    private AutoTrajectoryPair toGrid;
    private AutoTrajectoryPair toCharge;

    @Override
    public Pose2d getStartPosition() {
        return toCube.getAllianceTrajectory().getInitialPose();
    }

    public SubConeCubeCharge(Arm arm, Claw claw, Drivetrain drivetrain) {
        toCube = new AutoTrajectoryPair(PathPlanner.loadPath("SubDriveToCube", new PathConstraints(1.75, 5), true));
        toGrid = new AutoTrajectoryPair(PathPlanner.loadPath("SubDriveToGrid", new PathConstraints(2,5)));
        toCharge = new AutoTrajectoryPair(PathPlanner.loadPath("SubGridToCharge", new PathConstraints(2, 4.5)));

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2),
            Commands.parallel(
                new DriveTrajectory(drivetrain, toCube::getAllianceTrajectory),
                Commands.sequence(
                    new WaitCommand(0.7),
                    arm.setPosition(ArmPosition.BACK),
                    claw.open()
                )
            ),
            claw.close(),
            new WaitCommand(0.3),
            arm.setPosition(ArmPosition.HOME),
            new DriveTrajectory(drivetrain, toGrid::getAllianceTrajectory),
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2),
            new DriveTrajectory(drivetrain, toCharge::getAllianceTrajectory),
            new WaitCommand(0.2)
        );
    }

}
