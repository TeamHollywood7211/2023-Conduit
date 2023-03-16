
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import static frc.robot.Constants.*;

public class LedSubsystem extends SubsystemBase {
    
    //define candle
    CANdle candle1 = new CANdle(LED_CANDLE);
    //create animations
    RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 1, 128);
    StrobeAnimation strobeAnimation = new StrobeAnimation(255, 0, 1);

    public LedSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness during init
        candle1.configAllSettings(config);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

	
    public void allPurple(){
        setAllLights(191,64,191,1);
    }

	
    public void allYellow(){
        setAllLights(255,240,0,1);
    }

	
    public void allRed(){
        setAllLights(255,0,0,1);
    }	
	

    public void allOff(){
        setAllLights(0,0,0,0);
        candle1.clearAnimation(0);        
    }    


    public void setAllLights(int red, int green, int blue, int bright)
    {
        // Make sure there isn't an animation overriding
        candle1.clearAnimation(0);
        // set brightness
        candle1.configBrightnessScalar(bright);
        // set color of all LEDs at once
        candle1.setLEDs(red, green, blue);
    }


    public void allRainbow(){
        rainbowAnimation.setNumLed(128);
        candle1.animate(rainbowAnimation);
    }


}