package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.*;

public class ArmSubsystem extends SubsystemBase {
  public enum armStates{HIGH, MID, LOW, STORED};
  private enum gripStates{CONE, CUBE, OUT};

  //private  angleCanCoder;

  private CANSparkMax armMotor;
  private CANSparkMax gripMotor;
  // private CANSparkMax leftGripWheelsMotor;
  // private CANSparkMax rightGripWheelsMotor;
  private SparkMaxPIDController armPID;
  private SparkMaxPIDController gripPID;

  public armStates armState;
  public gripStates gripState;

  private double newArmkP = armkP;
  private double newArmkI = armkI;
  private double newArmkD = armkD;

  private double newGripkP = gripkP;
  private double newGripkI = gripkI;
  private double newGripkD = gripkD;

  public ArmSubsystem() {
    armState = armStates.LOW;
    gripState = gripStates.OUT;

    gripMotor = new CANSparkMax(GRIP_MOTOR_ID, MotorType.kBrushless);
    gripPID = gripMotor.getPIDController();

    //angleCanCoder = new CANCoder(52);

    // leftGripWheelsMotor = new CANSparkMax(LEFT_GRIP_WHEELS_MOTOR_ID, MotorType.kBrushless);
    // rightGripWheelsMotor = new CANSparkMax(RIGHT_GRIP_WHEELS_MOTOR_ID, MotorType.kBrushless);
    
    armMotor = new CANSparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
    armPID = armMotor.getPIDController();

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

  public double getAnglePos(){
    return armMotor.getEncoder().getPosition();
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

  public void runGripIn(){
    gripMotor.set(.5);
  }

  public void runGripOut(){
    gripMotor.set(-.5);
  }

  public void stopGrip(){
    gripMotor.set(0);
  }

  public boolean armOutsideFramePerim(int offset){
    if(getAnglePos() > armLowTarget-1+offset){
      return true;
    }
    return false;
  }

  // public void runGripWheels(){
  //   leftGripWheelsMotor.set(0.5);
  //   rightGripWheelsMotor.set(0.5);
  // }

  //this configures the motor controllers for the arm
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
    SmartDashboard.putNumber("ANGLE POS", getAnglePos());
    SmartDashboard.putNumber("GRIP POS", getGripPos());
    SmartDashboard.putBoolean("is outside frame", armOutsideFramePerim(0));
    
    // This method will be called once per scheduler run

    //gets the constants from the dashboard
    // newArmkP = SmartDashboard.getNumber("arm P", armkP);
    // newArmkI = SmartDashboard.getNumber("arm I", armkI);
    // newArmkD = SmartDashboard.getNumber("arm D", armkD);
    SmartDashboard.putNumber("Grip Current", gripMotor.getOutputCurrent());

    newGripkP = SmartDashboard.getNumber("grip P", gripkP);
    newGripkI = SmartDashboard.getNumber("grip I", gripkI);
    newGripkD = SmartDashboard.getNumber("grip D", gripkD);

    // armPID.setP(newArmkP);
    // armPID.setI(newArmkI);
    // armPID.setD(newArmkD);

    gripPID.setP(newGripkP);
    gripPID.setI(newGripkI);
    gripPID.setD(newGripkD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
