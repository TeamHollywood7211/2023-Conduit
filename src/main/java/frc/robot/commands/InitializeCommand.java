package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class InitializeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private CounterweightSubsystem m_counterweightSubsystem;
  private SolenoidSubsystem m_solenoidSubsystem;
  private boolean isDone;

  public InitializeCommand(ArmSubsystem armSubsystem, CounterweightSubsystem counterweightSubsystem, SolenoidSubsystem solenoidSubsystem) {
    m_armSubsystem = armSubsystem;
    m_counterweightSubsystem = counterweightSubsystem;
    m_solenoidSubsystem = solenoidSubsystem;
    addRequirements(armSubsystem, counterweightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.configureMotorControllers();
    //m_armSubsystem.setGripOut();
    m_counterweightSubsystem.configureCounterweightMotor();
    m_solenoidSubsystem.enableAnalogCompressor();
    m_solenoidSubsystem.retractWrist();
    m_solenoidSubsystem.retractArm();
    isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isDone){
      return true;
    }
    return false;
  }
}
