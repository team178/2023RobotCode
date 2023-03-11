package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand() {}

    public Pose2d getStartPosition() {
        return new Pose2d();
    }
}
