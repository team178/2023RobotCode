package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class ChargeOverCube extends AutoCommand {

    private AutoTrajectoryPair getOnCharge;
    private AutoTrajectoryPair getOffCharge;
    private AutoTrajectoryPair getBackOnCharge;

    @Override
    public Pose2d getStartPosition() {
        return getOnCharge.getAllianceTrajectory().getInitialPose();
    }

    public ChargeOverCube(Arm arm, Claw claw, Drivetrain drivetrain) {

        getOnCharge = new AutoTrajectoryPair(PathPlanner.loadPath("GoOverCharge", new PathConstraints(2, 5), true));
        getOffCharge = new AutoTrajectoryPair(PathPlanner.loadPath("GetOffCharge", new PathConstraints(0.75, 5), true));
        getBackOnCharge = new AutoTrajectoryPair(PathPlanner.loadPath("GetBackOnCharge", new PathConstraints(1, 5)));

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new DriveTrajectory(drivetrain, getOnCharge::getAllianceTrajectory),
            new DriveTrajectory(drivetrain, getOffCharge::getAllianceTrajectory),
            new DriveTrajectory(drivetrain, getBackOnCharge::getAllianceTrajectory)
        );
    }

}
