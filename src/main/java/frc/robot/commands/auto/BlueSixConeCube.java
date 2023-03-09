package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmPosition;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class BlueSixConeCube extends AutoCommand {

    public Pose2d getStartPosition() {
       return new Pose2d(1.83, 4.94, new Rotation2d(Units.degreesToRadians(180)));
    }

    public BlueSixConeCube(Arm arm, Claw claw, Drivetrain drivetrain, RamseteAutoBuilder autoBuilder) {
        this.addRequirements(
            arm, claw, drivetrain
        );

        PathPlannerTrajectory back = PathPlanner.loadPath("TestPath", new PathConstraints(1.75,5));
        PathPlannerTrajectory finish = PathPlanner.loadPath("DriveUpToCubeGridWithCube", new PathConstraints(2,5));
        
        drivetrain.resetPose(getStartPosition());

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
