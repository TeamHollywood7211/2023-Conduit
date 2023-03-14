package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

public class ArmHomeAuton extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  ArmSubsystem m_armSubsystem;
  SolenoidSubsystem m_solenoidSubsystem;
  Timer time;
  WaitCommand wait;

  public ArmHomeAuton(ArmSubsystem armSubsystem, SolenoidSubsystem solenoidSubsystem) {
    m_armSubsystem = armSubsystem;
    m_solenoidSubsystem = solenoidSubsystem;
    time = new Timer();
    
    addRequirements(armSubsystem, solenoidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_solenoidSubsystem.retractWrist();
    m_armSubsystem.setArmStored();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time.get()>0.35){
      return true;
    }
    return false;
  }
}
