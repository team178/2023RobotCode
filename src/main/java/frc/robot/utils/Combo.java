package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Combo {

    private final XboxController m_controller;
    private int stage = 0;

    private Timer m_timeout = new Timer();

    public Combo(XboxController controller) {
        m_controller = controller;
        m_timeout.start();

    }
    
    public Trigger quarterCircleKick() {
        return new Trigger(() -> {
            return poll();
        });
    }

    public boolean poll() {
        
        if (stage == 0 && m_controller.getPOV() == 180) {
            stage = 1;
            m_timeout.reset();
        } else if (m_timeout.hasElapsed(1)) {
            stage = 0;
        } else if (stage == 1 && m_controller.getPOV() == 135) {
            stage = 2;
            m_timeout.reset();
        } else if (stage == 2 && m_controller.getPOV() == 90) {
            stage = 3;
            m_timeout.reset();
        } else if (stage == 3 && m_controller.getPOV() == 90) {
            stage = 4;
            m_timeout.reset();
        } else if (stage == 4 && m_controller.getPOV() == 135) {
            stage = 5;
            m_timeout.reset();
        } else if (stage == 5 && m_controller.getPOV() == 90) {
            stage = 6;
            m_timeout.reset();
        } else if (stage == 6 && m_controller.getPOV() == 90) {
            stage = 7;
            m_timeout.reset();
        } else if (stage == 7 && m_controller.getLeftTriggerAxis() == 1) {
            stage = 0;
            return true;
        }
        
        return false;


    }

}
