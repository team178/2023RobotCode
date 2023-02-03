package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.kFwdChannel, ClawConstants.kRevChannel);

    public Claw() {
        m_solenoid.set(Value.kForward); // might need change depending on what desired starting value should be
    }

    public void toggle() {
        m_solenoid.toggle();
    }

    public Value getValue() {
        return m_solenoid.get();
    }
}
