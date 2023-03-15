package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.Autos;

public class AutoTrajectoryPair {
    private Trajectory redPath;
    private Trajectory bluePath;
    
    public AutoTrajectoryPair(Trajectory bluePath) {
        this.bluePath = bluePath;
        redPath = Autos.mirrorTrajectory(bluePath);
    }

    public AutoTrajectoryPair(Trajectory path, Alliance alliance) {
        if(alliance.equals(Alliance.Red)) {
            redPath = path;
            bluePath = Autos.mirrorTrajectory(redPath);
        } else {
            bluePath = path;
            redPath = Autos.mirrorTrajectory(bluePath);
        }
    }

    public Trajectory getAllianceTrajectory() {
        return DriverStation.getAlliance().equals(Alliance.Red) ? redPath : bluePath;
    }
}
