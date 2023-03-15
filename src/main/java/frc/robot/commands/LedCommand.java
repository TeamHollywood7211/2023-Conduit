package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;


public class LedCommand extends CommandBase {

  private final LedSubsystem m_Ledsubsystem;
  private CommandGenericHID m_controller; 


  public  LedCommand(LedSubsystem subsystem,CommandGenericHID m_joy) {
    m_Ledsubsystem = subsystem;
    m_controller = m_joy;
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if((m_controller.button(1).getAsBoolean()))
    {
      m_Ledsubsystem.allYellow();; 
    }
    if((m_controller.button(2).getAsBoolean()))
    {
      m_Ledsubsystem.allPurple();;
    }
    if((m_controller.button(3).getAsBoolean()))
    {
      m_Ledsubsystem.allRed();
    }
    if((m_controller.button(4).getAsBoolean()))
    {
      m_Ledsubsystem.allRainbow();
    }
    if((m_controller.button(5).getAsBoolean()))
    {
      m_Ledsubsystem.allOff();
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
