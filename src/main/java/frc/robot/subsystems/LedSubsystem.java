
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import static frc.robot.Constants.*;

public class LedSubsystem extends SubsystemBase {
    
    //define candle
    CANdle candle1 = new CANdle(LED_CANDLE_ID);
    int colorNum = 1;
    //create animations
    RainbowAnimation rainbowAnimation;
    StrobeAnimation strobeAnimation;
    ColorFlowAnimation redFlowAnimation;
    ColorFlowAnimation greenFlowAnimation;
    ColorFlowAnimation blueFlowAnimation;
    ColorFlowAnimation FlowAnimation2;
    ColorFlowAnimation FlowAnimation3;
    LarsonAnimation larsonAnimation;
    FireAnimation fireAnim;

    //Parts of enabledAnim
    LarsonAnimation backStripAnim;
    ColorFlowAnimation leftSideAnim;
    ColorFlowAnimation rightSideAnim;
    public boolean swagIsDone = false;

    public LedSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness during init
        candle1.configAllSettings(config);

        rainbowAnimation = new RainbowAnimation(1, 1, 128);
        rainbowAnimation.setNumLed(numLEDs);

        strobeAnimation = new StrobeAnimation(255, 0, 0);
        strobeAnimation.setNumLed(numLEDs);
        strobeAnimation.setSpeed(1);
        
        redFlowAnimation = new ColorFlowAnimation(255, 0, 0);
        redFlowAnimation.setNumLed(numLEDs);
        redFlowAnimation.setSpeed(0.5);

        blueFlowAnimation = new ColorFlowAnimation(0, 0, 255);
        blueFlowAnimation.setNumLed(188);
        blueFlowAnimation.setSpeed(0.7);
        blueFlowAnimation.setLedOffset(0);

        FlowAnimation2 = new ColorFlowAnimation(0, 0, 255);
        FlowAnimation2.setNumLed(188);
        FlowAnimation2.setSpeed(0.7);
        FlowAnimation2.setLedOffset(120);

        FlowAnimation3 = new ColorFlowAnimation(200, 0, 255);
        FlowAnimation3.setNumLed(200);
        FlowAnimation3.setSpeed(0.7);
        FlowAnimation3.setLedOffset(120);

        greenFlowAnimation = new ColorFlowAnimation(0, 255, 0);
        greenFlowAnimation.setNumLed(numLEDs);
        greenFlowAnimation.setSpeed(0.75);

        fireAnim = new FireAnimation();
        fireAnim.setNumLed(numLEDs);

        larsonAnimation = new LarsonAnimation(255, 0, 0);
        larsonAnimation.setNumLed(numLEDs);
        larsonAnimation.setSpeed(0.5);

        //Parts of enabledAnim
        backStripAnim = new LarsonAnimation(255, 0, 0);
        backStripAnim.setNumLed(26);
        backStripAnim.setLedOffset(107);
        backStripAnim.setSpeed(0.50);

        leftSideAnim = new ColorFlowAnimation(255, 0, 0);
        leftSideAnim.setNumLed(106);
        leftSideAnim.setSpeed(0.8);

        rightSideAnim = new ColorFlowAnimation(255, 0, 0);
        rightSideAnim.setLedOffset(134);
        rightSideAnim.setNumLed(106);
        rightSideAnim.setSpeed(0.8);
        rightSideAnim.setDirection(Direction.Backward);


    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

	public void incrementColor(){
        colorNum += 1;
        rainbowRumbleColor();
        //really bad if statement tree
    }
	public void deincrementColor(){
        colorNum -= 1;
        rainbowRumbleColor();
        //really bad if statement tree
    }

    public void rainbowRumbleColor(){
        if(colorNum > 6)
        {
            colorNum = 1;
        }
        if(colorNum < 1)
        {
            colorNum = 6;
        }

        if(colorNum == 1){allRed();}
        if(colorNum == 2){allYellow();}
        if(colorNum == 3){allOrange();}
        if(colorNum == 4){allGreen();}
        if(colorNum == 5){allBlue();}
        if(colorNum == 6){allPink();}
    }

    public void allPurple(){
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2); 
        setAllLights(191,0,191,1);
    }
    public void allPink(){
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2); 
        setAllLights(242,179,255,0.5);
    }

	
    public void allYellow(){
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2); 
        setAllLights(255,190,0,1);
    }

	
    public void allRed(){
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2); 
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
    public void allOrange(){
        setAllLights(128,42,0,0.5);
    }
    //*Red, *Yellow, *Orange, *Green, *Blue, *Purple

    public void allOff(){
        setAllLights(0,0,0,0);
        candle1.clearAnimation(0);        
    }    


    public void setAllLights(int red, int green, int blue, double bright)
    {
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2);
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

    public void allBlueFlow(){
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2);
        candle1.animate(blueFlowAnimation, 0);
        candle1.animate(FlowAnimation2, 1);
    }

    public void allRedBounce(){
        candle1.animate(larsonAnimation);
    }

    public void allGreenFlow(){
        candle1.animate(greenFlowAnimation);
    }

    public void allRedStrobe(){
        candle1.animate(strobeAnimation);
    }

    public void enabledAnim(){
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2);
        candle1.setLEDs(0, 0, 0);
        candle1.animate(leftSideAnim, 0);
        candle1.animate(rightSideAnim, 1);
        candle1.animate(backStripAnim, 2);
        allRainbow(); //COMMENT OUT AFTER RAINBOW RUMBLE!!
    }

    public void distanceLights(CameraSubsystem cameraSubsystem){
        candle1.clearAnimation(0);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2);
        if(cameraSubsystem.getFrontArea() < playerStationAreaTarget){
            candle1.setLEDs(100, 0, 0, 0, 108, (int)(4 + cameraSubsystem.getFrontArea()*7.5));
            candle1.setLEDs(0, 0, 0, 0, 0, 107);
            candle1.setLEDs(0, 0, 0, 0, 107 + (int)(5 + cameraSubsystem.getFrontArea()*7.5), 318);  
        } else if(cameraSubsystem.getFrontArea() > playerStationAreaTarget){
            allGreen();
        }
        // if(m_tofSubsystem.getDistance() > playerStationDistance + playerStationDistanceDeadzone || m_tofSubsystem.getDistance() < playerStationDistance - playerStationDistanceDeadzone){  
        //     candle1.setLEDs(100, 0, 0, 0, 108, 29 - (int)Math.round(m_tofSubsystem.getDistance()/100));
        //     candle1.setLEDs(0, 0, 0, 0, 0, 107);
        //     candle1.setLEDs(0, 0, 0, 0, 107 + 30 - (int)Math.round(m_tofSubsystem.getDistance()/100), 318);  
        // } else if(m_tofSubsystem.getDistance() < playerStationDistance + playerStationDistanceDeadzone && m_tofSubsystem.getDistance() > playerStationDistance - playerStationDistanceDeadzone){
        //     allGreen();
        // }
    }
}