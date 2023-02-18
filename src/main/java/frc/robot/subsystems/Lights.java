package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class Lights extends SubsystemBase {

  
  private AddressableLEDBuffer m_lightBarBuffer;
  private AddressableLED m_lightBar;

  public Lights() {
    initBar();
  }
  

  @Override
  public void periodic() {
    m_lightBar.setData(m_lightBarBuffer);
  }

  private void initBar() {
    m_lightBar = new AddressableLED(LightConstants.kLightBarPWMPort);
    m_lightBarBuffer = new AddressableLEDBuffer(LightConstants.kLightBarLength);
    m_lightBar.setLength(m_lightBarBuffer.getLength());
    m_lightBar.start();
  }

  private void setColor(Color c) {
    for(int i = 0; i < m_lightBarBuffer.getLength(); i++) {
      // Sets the specified LED to the Color inputted
      m_lightBarBuffer.setLED(i, c);
    }
  }
  
  private void blue() {
    setColor(new Color(0, 0, 0));
  }

  
  private void yellow() {
    setColor(new Color(255, 200, 0));
  }

  private void purple() {
    setColor(new Color(90, 0, 100));
  }

  public Command runBlue() {
    return run(() -> blue());
  }

  public Command runYellow() {
    return run(() -> yellow());
  }

  public Command runPurple() {
    return run(() -> purple());
  }
}