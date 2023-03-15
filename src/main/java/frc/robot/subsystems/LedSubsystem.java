
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import static frc.robot.Constants.*;

public class LedSubsystem extends SubsystemBase {
    
    //define candle
    CANdle candle1 = new CANdle(LED_CANDLE);
    //create a rainbow anim.
    RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 1, 128);

    public LedSubsystem() {

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
        setAllLights(80,45,127,1);
    }

	
    public void allYellow(){
        setAllLights(255,200,46,0);
    }

	
    public void allRed(){
        setAllLights(255,0,0,1);
    }	
	

    public void allOff(){
        setAllLights(0,0,0,0);
    }    


    public void setAllLights(int red, int green, int blue, int bright)
    {
        //set brightness
        candle1.configBrightnessScalar(0);
        //set color of all LEDs at once
        candle1.setLEDs(0, 0, 0);
    }


    public void allRainbow(){
        // This is still work in progress
        candle1.animate(rainbowAnimation);
    }


}