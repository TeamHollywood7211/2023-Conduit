package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;
import static frc.robot.Constants.*;

public class ArmToLowAuton extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ArmSubsystem m_armSubsystem;
  private SolenoidSubsystem m_solenoidSubsystem;

  public ArmToLowAuton(ArmSubsystem armSubsystem, SolenoidSubsystem solenoidSubsystem){
    m_armSubsystem = armSubsystem;
    m_solenoidSubsystem = solenoidSubsystem;
    addRequirements(armSubsystem, solenoidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmLow();
    if(m_armSubsystem.getArmPos() > armLowTarget-3){
      m_solenoidSubsystem.extendWrist();
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
