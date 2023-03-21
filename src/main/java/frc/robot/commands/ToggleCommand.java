package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ToggleCommand extends CommandBase {
  
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final CommandXboxController m_controller;
  private boolean isDone;
  public ToggleCommand(DrivetrainSubsystem drivetrainSubsystem, CommandXboxController controller) {
    m_controller = controller;
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    //this.ignoringDisable(true);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {isDone = false;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.start().getAsBoolean()){
      m_drivetrainSubsystem.toggleFieldOriented();
      isDone = true;  
    }
    if(m_controller.back().getAsBoolean()){
      m_drivetrainSubsystem.zeroGyroscope();
      isDone = true;
    }
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
