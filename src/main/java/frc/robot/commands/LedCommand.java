package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.ExampleFiles.ExampleSubsystem;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LedSubsystem m_Ledsubsystem;
  private CommandGenericHID m_controller = new CommandGenericHID(1); 


  public  LedCommand(LedSubsystem subsystem,CommandGenericHID m_coolBoard) {
    m_Ledsubsystem = subsystem;
    m_controller = m_coolBoard;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_controller.povUp().getAsBoolean())
    {
      m_Ledsubsystem.callCone();
    }
    if(m_controller.povDown().getAsBoolean())
    {
      m_Ledsubsystem.callCube();
    }
    if(m_controller.povRight().getAsBoolean())
    {
      m_Ledsubsystem.tasteTheRainbow();
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
