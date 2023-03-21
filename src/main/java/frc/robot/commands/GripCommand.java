package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripSubsystem;

import static frc.robot.Constants.*;

public class GripCommand extends CommandBase {
  private GripSubsystem m_gripSubsystem;
  private CommandXboxController m_controller;
  public GripCommand(GripSubsystem gripSubsystem, CommandXboxController controller) {
    m_controller = controller;
    m_gripSubsystem = gripSubsystem;
    addRequirements(gripSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getRightTriggerAxis() > gripTriggerDeadzone){
      m_gripSubsystem.runGripIn(m_controller.getRightTriggerAxis());
      //m_gripSubsystem.setGripCone();
    } else if(m_controller.getLeftTriggerAxis() > gripTriggerDeadzone && m_gripSubsystem.getGripPos() > 0.1){
      m_gripSubsystem.runGripOut(m_controller.getLeftTriggerAxis());
      //m_gripSubsystem.setGripCube();
    }
    else{
      //m_gripSubsystem.setGripOut();
      m_gripSubsystem.stopGrip();
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
