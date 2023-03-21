package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class UntipRobotAuton extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private Timer time;
  private boolean isDone;

  public UntipRobotAuton(DrivetrainSubsystem drivetrainSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    time = new Timer();
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.unpitchRobot();

    if((m_drivetrainSubsystem.getPitch() < 6 && m_drivetrainSubsystem.getPitch() > -6)){
      time.start();
    } else{
      time.reset();
    }

    if(time.get() >= 2.5){
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    time.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isDone){
      return true;
    }
    return false;
  }
}
