
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import static frc.robot.Constants.*;

public class LedSubsystem extends SubsystemBase {
    
    //define candle
    CANdle candle1 = new CANdle(LED_CANDLE_ID);
    //create animations
    RainbowAnimation rainbowAnimation;
    StrobeAnimation strobeAnimation;
    ColorFlowAnimation redFlowAnimation;
    ColorFlowAnimation greenFlowAnimation;
    LarsonAnimation larsonAnimation;
    public boolean swagIsDone = false;

    public LedSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness during init
        candle1.configAllSettings(config);

        rainbowAnimation = new RainbowAnimation(1, 1, 128);
        rainbowAnimation.setNumLed(numLEDs);

        strobeAnimation = new StrobeAnimation(0, 255, 0);
        strobeAnimation.setNumLed(numLEDs);
        strobeAnimation.setSpeed(0.6);
        
        redFlowAnimation = new ColorFlowAnimation(255, 0, 0);
        redFlowAnimation.setNumLed(numLEDs);
        redFlowAnimation.setSpeed(0.5);

        greenFlowAnimation = new ColorFlowAnimation(0, 255, 0);
        greenFlowAnimation.setNumLed(numLEDs);
        greenFlowAnimation.setSpeed(0.75);

        larsonAnimation = new LarsonAnimation(255, 0, 0);
        larsonAnimation.setNumLed(numLEDs);
        larsonAnimation.setSpeed(1.2);
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
        setAllLights(191,0,191,1);
    }

	
    public void allYellow(){
        setAllLights(255,190,0,1);
    }

	
    public void allRed(){
        setAllLights(255,0,0,1);
    }	
	

    public void allGreen(){
        setAllLights(0,255,0,1);
    }	


    public void allBlue(){
        setAllLights(0,0,255,0.5);
    }	


    public void allWhite(){
        setAllLights(255,255,255,0.5);
    }	

    public void allOff(){
        setAllLights(0,0,0,0);
        candle1.clearAnimation(0);        
    }    


    public void setAllLights(int red, int green, int blue, double bright)
    {
        // Make sure there isn't an animation overriding
        candle1.clearAnimation(0);
        // set brightness
        candle1.configBrightnessScalar(bright);
        // set color of all LEDs at once
        candle1.setLEDs(red, green, blue);
    }


    public void allRainbow(){
        candle1.animate(rainbowAnimation);
    }

    public void allRedFlow(){
        candle1.animate(redFlowAnimation);
    }

    public void allRedBounce(){
        candle1.animate(larsonAnimation);
    }

    public void allGreenFlow(){
        candle1.animate(greenFlowAnimation);
    }

    public void allGreenStrobe(){
        candle1.animate(strobeAnimation);
    }
}