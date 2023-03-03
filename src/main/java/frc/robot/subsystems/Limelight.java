package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class Limelight extends SubsystemBase {
    
    private NetworkTable table; //may not need it since we have limelight helper, if so can delete later

    private String name;

    private boolean isConnected = false;

    private LimelightResults limelightResults;

    public Limelight() {
        name = "limelight";
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    
    public Limelight(String name) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    public boolean isConnected() {
        return isConnected;
    }

    public Pose2d getPose() {
        return LimelightHelpers.getBotPose2d(name);
    }

    public double getTimestamp() {
        return limelightResults.targetingResults.timestamp_LIMELIGHT_publish; // might be the other timestamp in limelight helper
    }

    @Override
    public void periodic() {
        limelightResults = LimelightHelpers.getLatestResults(name);
    }
}
