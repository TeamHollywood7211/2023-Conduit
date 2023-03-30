package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class GripSubsystem extends SubsystemBase {
  private enum gripStates{CONE, CUBE, OUT};
  private gripStates gripState;

  private CANSparkMax gripMotor;
  private RelativeEncoder gripEncoder;
  private SparkMaxPIDController gripPID;


  /** Creates a new ExampleSubsystem. */
  public GripSubsystem() {
    gripMotor = new CANSparkMax(GRIP_MOTOR_ID, MotorType.kBrushless);
    gripPID = gripMotor.getPIDController();
    gripEncoder = gripMotor.getEncoder();

    gripPID.setP(gripkP);
    gripPID.setI(gripkI);
    gripPID.setD(gripkD);
  }

  public void setGripCone(){
    gripPID.setReference(gripConeTarget, ControlType.kPosition);
    gripState = gripStates.CONE;
  }

  public void setGripConeLoose(){
    gripPID.setReference(gripConeTarget-0.20, ControlType.kPosition);
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

  public void runGripOut(double input){
    gripMotor.set(-GRIP_MOTOR_SPEED*input);
  }

  public void runGripIn(double input){
    gripMotor.set(GRIP_MOTOR_SPEED*input);
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

  public double getGripTemp(){
    return gripMotor.getMotorTemperature();
  }

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

  public void configureMotorControllers(){
    gripMotor.restoreFactoryDefaults();

    gripMotor.setSmartCurrentLimit(GRIP_MOTOR_CURRENT_LIMIT);

    gripMotor.setIdleMode(IdleMode.kBrake);

    gripPID.setP(gripkP);
    gripPID.setI(gripkI);
    gripPID.setD(gripkD);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
