package frc.robot.commands;

public enum ArmPosition {
    // Arm positions in radians
    HOLD,
    HOME (0.171531, 5.999611),
    SUBSTATION(-1.142557, 5.177190),
    HIGH (-2.259555, 3.154062),
    LOW (-1.577659, 4.489911);

    public double lower;
    public double upper;

    private ArmPosition() {}

    private ArmPosition(double lower, double upper) {
        this.lower = lower;
        this.upper = upper;
    }
}