package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClawConstants.kChannel);
    private DigitalInput m_photosensor = new DigitalInput(2);

    public Claw() {
        m_solenoid.set(false);
    }

    public boolean getPhotosensor() {
        return !m_photosensor.get();
    }

    public CommandBase toggle() {
        return Commands.runOnce(() -> m_solenoid.toggle());
    }

    public Command close() {
        return Commands.runOnce(() -> m_solenoid.set(false));
    }

    public Command open() {
        return Commands.runOnce(() -> m_solenoid.set(true));
    }

    public void periodic() {
        SmartDashboard.putBoolean("photosensor", getPhotosensor());
    }

}
