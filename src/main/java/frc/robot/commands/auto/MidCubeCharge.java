package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class MidCubeCharge extends AutoCommand {

    private AutoTrajectoryPair getOnCharge;

    @Override
    public Pose2d getStartPosition() {
        return getOnCharge.getAllianceTrajectory().getInitialPose();
    }

    public MidCubeCharge(Arm arm, Claw claw, Drivetrain drivetrain) {
        getOnCharge = new AutoTrajectoryPair(PathPlanner.loadPath("MidGetOnCharge", new PathConstraints(2, 5), true));

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new DriveTrajectory(drivetrain, getOnCharge::getAllianceTrajectory)
        );
    }

}
