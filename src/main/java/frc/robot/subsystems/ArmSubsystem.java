// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.*;

public class ArmSubsystem extends SubsystemBase {
  public enum armStates{HIGH, MID, LOW};

  private CANSparkMax angleMotor;
  private CANSparkMax gripMotor;
  private CANSparkMax leftGripWheelsMotor;
  private CANSparkMax rightGripWheelsMotor;
  private SparkMaxPIDController armPID;

  public armStates armState;

  private double newkP = armkP;
  private double newkI = armkI;
  private double newkD = armkD;

  public ArmSubsystem() {
    armState = armStates.LOW;
    gripMotor = new CANSparkMax(GRIP_MOTOR_ID, MotorType.kBrushless);
    leftGripWheelsMotor = new CANSparkMax(LEFT_GRIP_WHEELS_MOTOR_ID, MotorType.kBrushless);
    rightGripWheelsMotor = new CANSparkMax(RIGHT_GRIP_WHEELS_MOTOR_ID, MotorType.kBrushless);
    
    angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, MotorType.kBrushless);
    armPID = angleMotor.getPIDController();

    //set angle arm pid constants
    armPID.setP(armkP);
    armPID.setI(armkI);
    armPID.setD(armkD);

    //put constants to the dashboard so we can tune them on the fly
    SmartDashboard.putNumber("arm P", armkP);
    SmartDashboard.putNumber("arm I", armkI);
    SmartDashboard.putNumber("arm D", armkD);
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

  //this configures the motor controllers for the arm
  public void configureMotorControllers(){
    angleMotor.restoreFactoryDefaults();
    gripMotor.restoreFactoryDefaults();
    leftGripWheelsMotor.restoreFactoryDefaults();
    rightGripWheelsMotor.restoreFactoryDefaults();

    angleMotor.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);
    gripMotor.setSmartCurrentLimit(GRIP_MOTOR_CURRENT_LIMIT);
    leftGripWheelsMotor.setSmartCurrentLimit(GRIP_WHEELS_MOTOR_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //gets the constants from the dashboard
    // newkP = SmartDashboard.getNumber("arm P", counterweightkP);
    // newkI = SmartDashboard.getNumber("arm I", counterweightkI);
    // newkD = SmartDashboard.getNumber("arm D", counterweightkD);

    // armPID.setP(newkP);
    // armPID.setI(newkI);
    // armPID.setD(newkD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
