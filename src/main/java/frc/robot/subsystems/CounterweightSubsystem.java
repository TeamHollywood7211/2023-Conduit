package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CounterweightSubsystem extends SubsystemBase {

  private CANSparkMax counterweightMotor; //assigns counterweight motor
  private SparkMaxPIDController counterweightPID; //assigns PID
  private double newkP = counterweightkP; //sets da p
  private double newkI = counterweightkI; //sets da i
  private double newkD = counterweightkD; //sets da D 

  public CounterweightSubsystem() {
    counterweightMotor = new CANSparkMax(COUNTERWEIGHT_MOTOR_ID, MotorType.kBrushless); //sets the motor to the cansparkmax motor
    counterweightPID = counterweightMotor.getPIDController(); //gets the PID controller
    counterweightPID.setP(counterweightkP); //actuallys sets da P
    counterweightPID.setI(counterweightkI); //actually sets da i
    counterweightPID.setD(counterweightkD); //actually sets da D

    SmartDashboard.putNumber("counterweight P", counterweightkP); 
    SmartDashboard.putNumber("counterweight I", counterweightkI);
    SmartDashboard.putNumber("counterweight D", counterweightkD);
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

    public void configureCounterweightMotor(){
      counterweightMotor.restoreFactoryDefaults();
      counterweightMotor.setSmartCurrentLimit(counterweightCurrentLimit);
      counterweightMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
      //this is run once every scheduler run
      SmartDashboard.putNumber("Counterweight Motor Position", counterweightMotor.getEncoder().getPosition());
  
      //sets constants to numbers from the dashboard
      // newkP = SmartDashboard.getNumber("counterweight P", counterweightkP);
      // newkI = SmartDashboard.getNumber("counterweight I", counterweightkI);
      // newkD = SmartDashboard.getNumber("counterweight D", counterweightkD);
  
      // counterweightPID.setP(newkP);
      // counterweightPID.setI(newkI);
      // counterweightPID.setD(newkD);
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }  
}
