package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class Lights extends SubsystemBase {

  
  private AddressableLEDBuffer m_lightBarBuffer;
  private AddressableLED m_lightBar;

  private Color color = new Color(0, 0, 0);
  private final Color pausedColor = new Color(255, 125, 0);
  private boolean pauseMainControl = false;

  public Lights() {
    initBar();
    switch(DriverStation.getAlliance()) {
      case Blue: color = new Color(0, 0, 255);
        break;
      case Red: color = new Color(255, 0, 0);
        break;
      default: color = new Color(0, 0, 0);
        break;
    }
  }
  

  @Override
  public void periodic() {
    if(pauseMainControl) {
      setColor((int) Timer.getFPGATimestamp() % 2 == 0 ? pausedColor : Color.kBlack);
    } else {
      setColor(color);
    }
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
  
  private void defaultColor() {
    Alliance a = DriverStation.getAlliance();
    switch(a) {
      case Blue: color = new Color(0, 0, 255);
        break;
      case Red: color = new Color(255, 0, 0);
        break;
      default: color = new Color(0, 0, 0);
        break;
    }
  }

  
  private void yellow() {
    color = new Color(255, 200, 0);
  }

  private void purple() {
    color = new Color(90, 0, 100);
  }

  public Command runDefaultColor() {
    return run(() -> defaultColor());
  }

  public Command runYellow() {
    return run(() -> yellow());
  }

  public Command runPurple() {
    return run(() -> purple());
  }

  public void setPauseMainControl(boolean pauseMainControl) {
      this.pauseMainControl = pauseMainControl;
  }
}