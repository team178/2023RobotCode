package frc.robot.commands.auto;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class ThreeSixConeCube extends AutoCommand {

    private Trajectory toCubeBlue;
    private Trajectory toGridBlue;

    private Trajectory toCubeRed;
    private Trajectory toGridRed;

    @Override
    public Pose2d getStartPosition() {
        return getCubeTrajectory().getInitialPose();
    }

    private Trajectory getCubeTrajectory() {
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            return toCubeRed;
        } else {
            return toCubeBlue;
        }
    }

    private Trajectory getGridTrajectory() {
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            return toGridRed;
        } else {
            return toGridBlue;
        }
    }

    public ThreeSixConeCube(Arm arm, Claw claw, Drivetrain drivetrain) {

        toCubeBlue = PathPlanner.loadPath("36DriveToCube", new PathConstraints(1.75, 5), true);
        toCubeRed = Autos.mirrorTrajectory(toCubeBlue);

        toGridBlue = PathPlanner.loadPath("36DriveToGrid", new PathConstraints(2,5));
        toGridRed = Autos.mirrorTrajectory(toGridBlue);

        this.addCommands(
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2),
            Commands.parallel(
                new DriveTrajectory(drivetrain, this::getCubeTrajectory),
                Commands.sequence(
                    new WaitCommand(0.7),
                    arm.setPosition(ArmPosition.BACK),
                    claw.open()
                )
            ),
            claw.close(),
            new WaitCommand(0.3),
            arm.setPosition(ArmPosition.HOME),
            new DriveTrajectory(drivetrain, this::getGridTrajectory),
            Autos.placeHigh(arm, claw),
            new WaitCommand(0.2)
        );
    }

}
