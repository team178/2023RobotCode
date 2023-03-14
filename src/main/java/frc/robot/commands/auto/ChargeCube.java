package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class ChargeCube extends AutoCommand {

    private Trajectory getOnChargeRed;
    private Trajectory getOnChargeBlue;

    private Trajectory getOffChargeRed;
    private Trajectory getOffChargeBlue;

    private Trajectory getBackOnChargeRed;
    private Trajectory getBackOnChargeBlue;

    @Override
    public Pose2d getStartPosition() {
        return getOnChargeTrajectory().getInitialPose();
    }

    private Trajectory getOnChargeTrajectory() {
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            return getOnChargeRed;
        } else {
            return getOnChargeBlue;
        }
    }

    private Trajectory getBackOnChargeTrajectory() {
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            return getBackOnChargeRed;
        } else {
            return getBackOnChargeBlue;
        }
    }

    private Trajectory getOffChargeTrajectory() {
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            return getOffChargeRed;
        } else {
            return getOffChargeBlue;
        }
    }

    public ChargeCube(Arm arm, Claw claw, Drivetrain drivetrain) {

        getOnChargeBlue = PathPlanner.loadPath("GetOnCharge", new PathConstraints(2, 5), true);
        getOnChargeRed = Autos.mirrorTrajectory(getOnChargeBlue);

        getOffChargeBlue = PathPlanner.loadPath("GetOffCharge", new PathConstraints(0.75, 5), true);
        getOffChargeRed = Autos.mirrorTrajectory(getOffChargeBlue);

        getBackOnChargeBlue = PathPlanner.loadPath("GetBackOnCharge", new PathConstraints(1, 5));
        getBackOnChargeRed = Autos.mirrorTrajectory(getBackOnChargeBlue);

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new DriveTrajectory(drivetrain, this::getOnChargeTrajectory),
            new DriveTrajectory(drivetrain, this::getOffChargeTrajectory),
            new DriveTrajectory(drivetrain, this::getBackOnChargeTrajectory)
        );
    }

}
