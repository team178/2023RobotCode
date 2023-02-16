package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pulse extends SubsystemBase
{

  public Pulse() {
    //initStrip(0, 60);
    initBar(2, 8);
  }
  
  private AddressableLED m_lightBar;
  private AddressableLEDBuffer m_lightBarBuffer;

  @Override
  public void periodic() {
    runRedBlue();
    m_lightBar.setData(m_lightBarBuffer);

  }

  private void initBar(int port, int length) {
    m_lightBar = new AddressableLED(port);
    m_lightBarBuffer = new AddressableLEDBuffer(length);
    m_lightBar.setLength(length);
    m_lightBar.start();
  }

  
  private void setColors(AddressableLEDBuffer lightBuffer, Color... c) {
    for(int i = 0; i < lightBuffer.getLength(); i++) {
      lightBuffer.setLED(i, c[i % c.length]);
    }
  }

  private void lightsOff(AddressableLEDBuffer lightBuffer) {
    setColors(lightBuffer, new Color(0, 0, 0));
  }

  public Command runRedBlue(){
    return startEnd(
      () -> setColors(m_lightBarBuffer, new Color(255, 0, 0), new Color(0, 0, 255)),
      () -> lightsOff(m_lightBarBuffer));
  }

  public Command runLightsOff() {
    return run(() -> lightsOff(m_lightBarBuffer));
  }

}