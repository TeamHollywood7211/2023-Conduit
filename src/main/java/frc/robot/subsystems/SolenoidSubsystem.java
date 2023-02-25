// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidSubsystem extends SubsystemBase {
  boolean toggleIntake = true;
  private boolean armSolenoidState;
  private boolean wristSolenoidState;
  /** Creates a new ExampleSubsystem. */
  private PneumaticHub m_pneumaticHub;
  private DoubleSolenoid m_armSolenoid;
  private DoubleSolenoid m_wristSolenoid;
  private Solenoid m_flipperSolenoid;

  public SolenoidSubsystem() {
    armSolenoidState = false;
    wristSolenoidState = false;
    m_pneumaticHub = new PneumaticHub(61);
    //m_robotCompressor = new Compressor(PneumaticsModuleType.REVPH);
    m_armSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 0, 1);
    //m_armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    //m_wristSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
    m_wristSolenoid = new DoubleSolenoid(61, PneumaticsModuleType.REVPH, 6, 7);
    m_flipperSolenoid = new Solenoid(61, PneumaticsModuleType.REVPH, 4);
  }

  public void enableAnalogCompressor(){
    m_pneumaticHub.enableCompressorAnalog(70, 120);
  }

  //method sets main extension solenoid to retract (run in initialization of command to make toggle work)
  public void retractArm(){
    m_armSolenoid.set(Value.kReverse);
    armSolenoidState = false;
  }

  public void extendArm(){
    m_armSolenoid.set(Value.kForward);
    armSolenoidState = true;
  }

  //returns false if retracted, true if extended
  public boolean getArmSolenoidState(){
    return armSolenoidState;
  }

  //method sets wrist solenoid to retract (also set in initialization)
  public void retractWrist(){
    m_wristSolenoid.set(Value.kReverse);
    wristSolenoidState = false;
  }

  public void extendWrist(){
    m_wristSolenoid.set(Value.kForward);
    wristSolenoidState = true;
  }

  public void toggleWrist(){
    m_wristSolenoid.toggle();
  }

  public boolean getWristSolenoidState(){
    return wristSolenoidState;
  }

  public void fireFlipperSolenoid(){
    m_flipperSolenoid.set(true);
  }

  public boolean getFlipperSolenoidState(){
    return m_flipperSolenoid.get();
  }

  public void disableFlipperSolenoid(){
    m_flipperSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Compressor State", m_robotCompressor.isEnabled());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
