package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.Constants.*;

public class GripCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private CommandXboxController m_controller;
  public GripCommand(ArmSubsystem armSubsystem, CommandXboxController controller) {
    m_controller = controller;
    m_armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getRightTriggerAxis() > gripTriggerDeadzone){
      m_armSubsystem.runGripIn();
      //m_armSubsystem.setGripCone();
    } else if(m_controller.getLeftTriggerAxis() > gripTriggerDeadzone){
      m_armSubsystem.runGripOut();
      //m_armSubsystem.setGripCube();
    }
    else{
      //m_armSubsystem.setGripOut();
      m_armSubsystem.stopGrip();
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
