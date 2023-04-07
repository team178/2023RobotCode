package frc.robot.commands;

public enum ArmPosition {
    // Arm positions in radians
    HOLD,
    HOME (5.407518, 5.999611),
    SUBSTATION(4.093430, 5.156882),
    HIGH (2.976432, 3.154062),
    LOW (3.587507, 4.466632),
    BACK (5.407518, 2.951937);

    public double lower;
    public double upper;

    private ArmPosition() {}

    private ArmPosition(double lower, double upper) {
        this.lower = lower;
        this.upper = upper;
    }
}