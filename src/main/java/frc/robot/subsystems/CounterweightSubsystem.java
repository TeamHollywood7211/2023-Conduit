package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CounterweightSubsystem extends SubsystemBase {

  private CANSparkMax counterweightMotor;
  private SparkMaxPIDController counterweightPID;
  private double newkP = counterweightkP;
  private double newkI = counterweightkI;
  private double newkD = counterweightkD;

  public CounterweightSubsystem() {
    counterweightMotor = new CANSparkMax(COUNTERWEIGHT_MOTOR_ID, MotorType.kBrushless);
    counterweightPID = counterweightMotor.getPIDController();
    counterweightPID.setP(counterweightkP);
    counterweightPID.setI(counterweightkI);
    counterweightPID.setD(counterweightkD);

    SmartDashboard.putNumber("counterweight P", counterweightkP);
    SmartDashboard.putNumber("counterweight I", counterweightkI);
    SmartDashboard.putNumber("counterweight D", counterweightkD);
  }

  @Override
  public void periodic() {
    //this is run once every scheduler run
    
    //sets constants to numbers from the dashboard
    newkP = SmartDashboard.getNumber("counterweight P", counterweightkP);
    newkI = SmartDashboard.getNumber("counterweight I", counterweightkI);
    newkD = SmartDashboard.getNumber("counterweight D", counterweightkD);

    counterweightPID.setP(newkP);
    counterweightPID.setI(newkI);
    counterweightPID.setD(newkD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    //this sets the arm to the high position, or top nodes
    public void setCounterweightHigh(){
      counterweightPID.setReference(counterweightHighTarget, ControlType.kPosition);
    }
  
    //this sets the arm to the mid position, or middle nodes
    public void setCounterweightMid(){
      counterweightPID.setReference(counterweightMidTarget, ControlType.kPosition);
    }
  
    //this sets the arm to the low position, or lowest nodes
    public void setCounterweightLow(){
      counterweightPID.setReference(counterweightLowTarget, ControlType.kPosition);
    }
  
}
