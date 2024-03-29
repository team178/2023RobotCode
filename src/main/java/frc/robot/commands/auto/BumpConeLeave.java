package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class BumpConeLeave extends AutoCommand {
    private AutoTrajectoryPair toCube;

    @Override
    public Pose2d getStartPosition() {
        return toCube.getAllianceTrajectory().getInitialPose();
    }

    public BumpConeLeave(Arm arm, Claw claw, Drivetrain drivetrain) {

        toCube = new AutoTrajectoryPair(PathPlanner.loadPath("BumpDriveToCube", new PathConstraints(1.0, 5), true));

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new WaitCommand(1),
            new DriveTrajectory(drivetrain, toCube::getAllianceTrajectory)
        );
    }
}
