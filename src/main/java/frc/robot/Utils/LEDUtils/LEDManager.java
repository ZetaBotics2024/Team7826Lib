package frc.robot.Utils.LEDUtils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LEDConstants;

public class LEDManager {
    private static AddressableLED LEDStrip;
    private static AddressableLEDBuffer LEDBuffer;

    public static void init() {
        LEDStrip = new AddressableLED(LEDConstants.kLEDPortID);
        LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);
        LEDStrip.setLength(LEDBuffer.getLength());
        LEDStrip.setData(LEDBuffer);
        LEDStrip.start(); 
    }

    public static void setSolidColor(int[] rgbColor) {
        for(int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, rgbColor[0],
            rgbColor[1],
            rgbColor[2]);
        }
        LEDStrip.setData(LEDBuffer);
    }

    public static void setPatern(int[] firstRGBColor, int[] secondRGBColor, int sectionLength) {
        int numberOfSections = (int)(LEDBuffer.getLength() / sectionLength);
        for(int i = 0; i < numberOfSections; i++) {
            for(int j = 0; j < sectionLength; j++) {
                if(i % 2 == 1) {
                    LEDBuffer.setRGB(i, firstRGBColor[0], firstRGBColor[1], firstRGBColor[2]);
                } else {
                    LEDBuffer.setRGB(i, secondRGBColor[0], secondRGBColor[1], secondRGBColor[2]);
                }
            }
        }
        LEDStrip.setData(LEDBuffer);
    }
    
}
