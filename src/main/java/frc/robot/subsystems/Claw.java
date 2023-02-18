package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClawConstants.kChannel);

    public Claw() {
        m_solenoid.set(false);
    }

    public void toggle() {
        m_solenoid.toggle();
    }

}
