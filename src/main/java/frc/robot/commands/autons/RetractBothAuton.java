package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SolenoidSubsystem;

public class RetractBothAuton extends CommandBase {
  private SolenoidSubsystem solenoidSubsystem;
  private Timer time;

  public RetractBothAuton(SolenoidSubsystem solenoidSubsystem) {
    this.solenoidSubsystem = solenoidSubsystem;
    time = new Timer();
    addRequirements(solenoidSubsystem);
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
    solenoidSubsystem.retractArm();
    solenoidSubsystem.retractWrist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time.get()>0.7){
      return true;
    }
    return false;
  }
}
