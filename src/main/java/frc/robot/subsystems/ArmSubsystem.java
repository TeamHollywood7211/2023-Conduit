package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.*;

public class ArmSubsystem extends SubsystemBase {
  public enum armStates{HIGH, MID, LOW, STORED};
  //private  angleCanCoder;

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armPID;

  public armStates armState;

  // private double newArmkP = armkP;
  // private double newArmkI = armkI;
  // private double newArmkD = armkD;

  public ArmSubsystem() {
    armState = armStates.STORED;
    
    armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
    armPID = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();

    //set angle arm pid constants
    armPID.setP(armkP);
    armPID.setI(armkI);
    armPID.setD(armkD);
    armPID.setSmartMotionAllowedClosedLoopError(1, 0);
  }

  //this sets the arm to the high position, or top nodes
  public void setArmHigh(){
    armPID.setReference(armHighTarget, ControlType.kPosition);
    armState = armStates.HIGH;
  }

  //this sets the arm to the mid position, or middle nodes
  public void setArmMid(){
    armPID.setReference(armMidTarget, ControlType.kPosition);
    armState = armStates.MID;
  }

  //this sets the arm to the low position, or lowest nodes
  public void setArmLow(){
    armPID.setReference(armLowTarget, ControlType.kPosition);
    armState = armStates.LOW;
  }

  public void setArmStored(){
    armPID.setReference(armStoredTarget, ControlType.kPosition);
    armState = armStates.STORED;
  }

  public void manualArmAdjust(double input){
    double alteredInput = -input*MANUAL_ARM_ADJUST_POWER_MULTIPLIER;
    double currentPos = armEncoder.getPosition();
    armPID.setReference(currentPos+alteredInput, ControlType.kPosition);
  }

  public void setArmJustAboveLow(){
    armPID.setReference(armLowTarget+12, ControlType.kPosition);
  }

  public void setArmJustBelowLow(){
    armPID.setReference(30, ControlType.kPosition);
  }

  public void driveArmBack(){
    armMotor.set(-.20);
  }

  public double getArmPos(){
    return armMotor.getEncoder().getPosition();
  }

  public double getArmCurrent(){
    return armMotor.getOutputCurrent();
  }

  public armStates getArmState(){
    return armState;
  }

  public boolean armIsDown(){
    if(getArmState() == armStates.LOW || getArmState() == armStates.STORED){
      return true;
    } else{
      return false;
    }
  }

  /**
   * 
   * @param offset from the frame perimeter, pos number makes it further out, negative makes it closer
   * @return true when the arm is outside frame perimeter false when it's inside
   */
  public boolean armOutsideFramePerim(int offset){
    if(getArmPos() > 33+offset){
      return true;
    }
    return false;
  }

  /**
   * Initializes the arm motor.
   * Runs the motor until it's pushing against the robot.
   * Once it surpasses some current and velocity limits, stop and set zero. 
   */
  //FIXME
  public void initializeArmMotor(){
    armMotor.set(-0.20);
    if(armMotor.getOutputCurrent() >= ARM_MOTOR_INIT_CURRENT_LIMIT){
      armMotor.stopMotor();
      armEncoder.setPosition(0);
    }
  }

  public void configureMotorControllers(){
    armMotor.restoreFactoryDefaults();

    armMotor.setSmartCurrentLimit(ARM_MOTOR_CURRENT_LIMIT);

    armMotor.setIdleMode(IdleMode.kBrake);

    armPID.setP(armkP);
    armPID.setI(armkI);
    armPID.setD(armkD);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Arm Pos", getArmPos());

    //gets the constants from the dashboard
    // newArmkP = SmartDashboard.getNumber("arm P", armkP);
    // newArmkI = SmartDashboard.getNumber("arm I", armkI);
    // newArmkD = SmartDashboard.getNumber("arm D", armkD);

    // armPID.setP(newArmkP);
    // armPID.setI(newArmkI);
    // armPID.setD(newArmkD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
