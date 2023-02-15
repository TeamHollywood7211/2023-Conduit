// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

//
//!!HEY!! BEFORE YOU GO DEEP INTO THIS LIKE 40 
//

public class LedSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax ledMotor;
    
  public LedSubsystem()
  {
    //ledMotor = new CANSparkMax(ledMotor, MotorType.kBrushless);
    //on god, no idea why that dont work, commenting it out (true nu cap moment)
  } 

  public void setMode(LEDMode mode)
  {

  }


  public enum LEDMode{

  }
  public void FlashCone() {
    //ledPWM.set(-0.09);

    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //ledPWM.set(currentLedMode);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
