/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class RGBstrip extends SubsystemBase {
  /**
   * Creates a new RGBstrip.
   */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_LedBuffer;
  public int m_rainbowFirstPixelHue;

public RGBstrip() {
    m_led = new AddressableLED (Constants.ledPort);
    m_LedBuffer = new AddressableLEDBuffer(150);
    m_led.setLength(m_LedBuffer.getLength());

    m_led.setData(m_LedBuffer);
    m_led.start();
  }

  public void rainbow(){
    for (var i = 0;  i < m_LedBuffer.getLength(); i++){
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LedBuffer.getLength())) % 180;
      m_LedBuffer.setHSV (i, hue, 225, 128);
    }
    m_rainbowFirstPixelHue += 3; 
    m_rainbowFirstPixelHue %= 180; 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
