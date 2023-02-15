package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.*;

public class ArmSubsystem extends SubsystemBase {
  public enum armStates{HIGH, MID, LOW, STORED};
  private enum gripStates{CONE, CUBE, OUT};

  //private  angleCanCoder;

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private CANSparkMax gripMotor;
  private RelativeEncoder gripEncoder;
  // private CANSparkMax leftGripWheelsMotor;
  // private CANSparkMax rightGripWheelsMotor;
  private SparkMaxPIDController armPID;
  private SparkMaxPIDController gripPID;

  public armStates armState;
  public gripStates gripState;

  private double newArmkP = armkP;
  private double newArmkI = armkI;
  private double newArmkD = armkD;

  // private double newGripkP = gripkP;
  // private double newGripkI = gripkI;
  // private double newGripkD = gripkD;

  public ArmSubsystem() {
    armState = armStates.LOW;
    gripState = gripStates.OUT;

    gripMotor = new CANSparkMax(GRIP_MOTOR_ID, MotorType.kBrushless);
    gripPID = gripMotor.getPIDController();
    gripEncoder = gripMotor.getEncoder();

    //angleCanCoder = new CANCoder(52);

    // leftGripWheelsMotor = new CANSparkMax(LEFT_GRIP_WHEELS_MOTOR_ID, MotorType.kBrushless);
    // rightGripWheelsMotor = new CANSparkMax(RIGHT_GRIP_WHEELS_MOTOR_ID, MotorType.kBrushless);
    
    armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
    armPID = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();

    //set angle arm pid constants
    armPID.setP(armkP);
    armPID.setI(armkI);
    armPID.setD(armkD);
    armPID.setSmartMotionAllowedClosedLoopError(1, 0);

    gripPID.setP(gripkP);
    gripPID.setI(gripkI);
    gripPID.setD(gripkD);

    //put constants to the dashboard so we can tune them on the fly
    SmartDashboard.putNumber("arm P", armkP);
    SmartDashboard.putNumber("arm I", armkI);
    SmartDashboard.putNumber("arm D", armkD);
    
    // //put constants to the dashboard so we can tune them on the fly
    SmartDashboard.putNumber("grip P", gripkP);
    SmartDashboard.putNumber("grip I", gripkI);
    SmartDashboard.putNumber("grip D", gripkD);
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

  public void driveArmBack(){
    armMotor.set(-.20);
  }

  public double getAnglePos(){
    return armMotor.getEncoder().getPosition();
  }

  public double getArmCurrent(){
    return armMotor.getOutputCurrent();
  }


  public void setGripCone(){
    gripPID.setReference(gripConeTarget, ControlType.kPosition);
    gripState = gripStates.CONE;
  }

  public void setGripCube(){
    gripPID.setReference(gripCubeTarget, ControlType.kPosition);
    gripState = gripStates.CUBE;
  }

  public void setGripOut(){
    gripPID.setReference(gripOutTarget, ControlType.kPosition);
    gripState = gripStates.OUT;
  }

  public double getGripPos(){
    return gripMotor.getEncoder().getPosition();
  }

  public void runGripOut(){
    gripMotor.set(0.35);
  }

  public void runGripIn(){
    gripMotor.set(-0.35);
  }

  public void runGripInPrecise(){
    gripMotor.set(-0.25);
  }

  public void stopGrip(){
    gripMotor.set(0);
  }

  public double getGripCurrent(){
    return gripMotor.getOutputCurrent();
  }

  /**
   * 
   * @param offset from the frame perimeter, pos number makes it further out, negative makes it closer
   * @return true when the arm is outside frame perimeter false when it's inside
   */
  public boolean armOutsideFramePerim(int offset){
    if(getAnglePos() > 33+offset){
      return true;
    }
    return false;
  }

  // public void runGripWheels(){
  //   leftGripWheelsMotor.set(0.5);
  //   rightGripWheelsMotor.set(0.5);
  // }

  /**
   * Initializes the grip motor.
   * Runs the motor until it's all the way closed.
   * Once current spikes above a specified amount, and velocity drops below a specified amount, set zero.
   */
  //FIXME
  /*Heres the idea, write code in the periodic section to check if the arm/counterweight motors have breached their current limit amt and the init button has been pressed in a certain time.
   * write the code in robotcontainer (boolean that returns true if a timer that is started when you press the init button is less than a certain amount) init motors button should set the motors backwards at a particular speed so we know the current limit to set
   */
  public void initializeGripMotor(){
    gripMotor.set(-1);
    if(
        (gripEncoder.getVelocity() <= GRIP_MOTOR_INIT_VELOCITY_MIN) 
        && 
        (gripMotor.getOutputCurrent() >= GRIP_MOTOR_INIT_CURRENT_LIMIT)
      ){
        gripMotor.set(0);
        gripEncoder.setPosition(0);
    }
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
    gripMotor.restoreFactoryDefaults();
    // leftGripWheelsMotor.restoreFactoryDefaults();
    // rightGripWheelsMotor.restoreFactoryDefaults();

    armMotor.setSmartCurrentLimit(ARM_MOTOR_CURRENT_LIMIT);
    gripMotor.setSmartCurrentLimit(GRIP_MOTOR_CURRENT_LIMIT);
    // leftGripWheelsMotor.setSmartCurrentLimit(GRIP_WHEELS_MOTOR_CURRENT_LIMIT);
    // rightGripWheelsMotor.setSmartCurrentLimit(GRIP_WHEELS_MOTOR_CURRENT_LIMIT);

    armMotor.setIdleMode(IdleMode.kBrake);
    gripMotor.setIdleMode(IdleMode.kBrake);
    // leftGripWheelsMotor.setIdleMode(IdleMode.kBrake);
    // rightGripWheelsMotor.setIdleMode(IdleMode.kBrake);

    armPID.setP(armkP);
    armPID.setI(armkI);
    armPID.setD(armkD);

    gripPID.setP(gripkP);
    gripPID.setI(gripkI);
    gripPID.setD(gripkD);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run

    //gets the constants from the dashboard
    newArmkP = SmartDashboard.getNumber("arm P", armkP);
    newArmkI = SmartDashboard.getNumber("arm I", armkI);
    newArmkD = SmartDashboard.getNumber("arm D", armkD);

    // newGripkP = SmartDashboard.getNumber("grip P", gripkP);
    // newGripkI = SmartDashboard.getNumber("grip I", gripkI);
    // newGripkD = SmartDashboard.getNumber("grip D", gripkD);

    armPID.setP(newArmkP);
    armPID.setI(newArmkI);
    armPID.setD(newArmkD);

    // gripPID.setP(newGripkP);
    // gripPID.setI(newGripkI);
    // gripPID.setD(newGripkD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
