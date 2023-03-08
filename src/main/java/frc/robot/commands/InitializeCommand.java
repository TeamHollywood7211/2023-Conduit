package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterweightSubsystem;

public class InitializeCommand extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private CounterweightSubsystem m_counterweightSubsystem;
  private CommandXboxController m_controller;

  public InitializeCommand(ArmSubsystem armSubsystem, CounterweightSubsystem counterweightSubsystem, CommandXboxController controller) {
    m_armSubsystem = armSubsystem;
    m_counterweightSubsystem = counterweightSubsystem;
    m_controller = controller;
    addRequirements(armSubsystem, counterweightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.configureMotorControllers();
    m_counterweightSubsystem.configureCounterweightMotor();
    
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
