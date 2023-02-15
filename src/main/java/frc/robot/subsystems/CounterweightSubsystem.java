package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CounterweightSubsystem extends SubsystemBase {

  private CANSparkMax counterweightMotor; //assigns counterweight motor
  private SparkMaxPIDController counterweightPID; //assigns PID
  private RelativeEncoder counterweightEncoder;

  public CounterweightSubsystem() {
    counterweightMotor = new CANSparkMax(COUNTERWEIGHT_MOTOR_ID, MotorType.kBrushless); //sets the motor to the cansparkmax motor
    counterweightEncoder = counterweightMotor.getEncoder();
    counterweightPID = counterweightMotor.getPIDController(); //gets the PID controller
    counterweightPID.setP(counterweightkP); //actuallys sets da P
    counterweightPID.setI(counterweightkI); //actually sets da i
    counterweightPID.setD(counterweightkD); //actually sets da D
  }

    //this sets the weight to the high position (as in when the arm is reaching for high things)
    public void setCounterweightHigh(){
      counterweightPID.setReference(counterweightHighTarget, ControlType.kPosition);
    }
  
    //this sets the weight to the mid position (as in for when the arm is reaching for mid things)
    public void setCounterweightMid(){
      counterweightPID.setReference(counterweightMidTarget, ControlType.kPosition);
    }
  
    //this sets the counterweight to the low position (as in for when the arm is out for the lowest spot)
    public void setCounterweightLow(){
      counterweightPID.setReference(counterweightLowTarget, ControlType.kPosition);
    }

    //this sets the counterweight to the stored position (as in for when the arm is all the way in)
    public void setCounterweightStored(){
      counterweightPID.setReference(counterweightLowTarget, ControlType.kPosition);
    }

    public double getCounterweightCurrent(){
      return counterweightMotor.getOutputCurrent();
    }

    public double getCounterweightPos(){
      return counterweightEncoder.getPosition();
    }

    /**
    * Initializes the counterweight motor.
    * Runs the motor until it's all the way closed, then once current spikes above 
    */
    //FIXME
    public void initializeCounterweightMotor(){
      counterweightMotor.set(0.5);
      if(counterweightMotor.getOutputCurrent() >= COUNTERWEIGHT_INIT_CURRENT_LIMIT){
        counterweightMotor.stopMotor();
        counterweightEncoder.setPosition(0);  
      }
    }

    /**
     * Allows for manual control of the counterweight for weight balance testing
     */
    public void manualCounterweightAdjust(double increment){
      double currentPos = counterweightEncoder.getPosition();
      counterweightPID.setReference(currentPos+increment, ControlType.kPosition);
    }

    public void configureCounterweightMotor(){
      counterweightMotor.restoreFactoryDefaults();
      counterweightMotor.setSmartCurrentLimit(COUNTERWEIGHT_CURRENT_LIMIT);
      counterweightMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
      //this is run once every scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }  
}
