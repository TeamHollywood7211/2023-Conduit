package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CounterweightSubsystem;

public class ManualCounterweightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private CounterweightSubsystem m_counterweightSubsystem;
  private CommandXboxController m_controller;


  public ManualCounterweightCommand(CounterweightSubsystem counterweightSubsystem, CommandXboxController controller) {
    m_counterweightSubsystem = counterweightSubsystem;
    m_controller = controller;
    addRequirements(counterweightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.povUp().getAsBoolean()){
      m_counterweightSubsystem.manualCounterweightAdjust(1);
    }
    if(m_controller.povDown().getAsBoolean()){
      m_counterweightSubsystem.manualCounterweightAdjust(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
