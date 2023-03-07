package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class GrabCubeAuton extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SolenoidSubsystem m_solenoidSubsystem;
  private GripSubsystem m_gripSubsystem;

  public GrabCubeAuton(SolenoidSubsystem solenoidSubsystem, GripSubsystem gripSubsystem){
    m_solenoidSubsystem = solenoidSubsystem;
    m_gripSubsystem = gripSubsystem;
    addRequirements(solenoidSubsystem, gripSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_gripSubsystem.setGripCube();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
