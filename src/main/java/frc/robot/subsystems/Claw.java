package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClawConstants.kChannel);
    private AnalogPotentiometer m_ultrasonic = new AnalogPotentiometer(0, 6);

    public Claw() {
        m_solenoid.set(false);
    }

    public double getUltrasonicDistance() {
        return m_ultrasonic.get();
    }

    public void toggle() {
        m_solenoid.toggle();
    }

    public Command close() {
        return Commands.runOnce(() -> m_solenoid.set(false));
    }

    public Command open() {
        return Commands.runOnce(() -> m_solenoid.set(true));
    }

}
