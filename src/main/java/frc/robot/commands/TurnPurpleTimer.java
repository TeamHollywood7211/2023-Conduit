package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;

public class TurnPurpleTimer extends CommandBase {
  Timer time;
  LedSubsystem m_ledSubsystem;

  public TurnPurpleTimer(LedSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
    time = new Timer();
    addRequirements();
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
    m_ledSubsystem.allPurple();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    time.reset();
    m_ledSubsystem.enabledAnim();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time.get()>4){
      return true;
    }
    return false;
  }
}
