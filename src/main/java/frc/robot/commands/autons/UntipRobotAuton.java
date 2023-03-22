package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LedSubsystem;
import static frc.robot.Constants.*;

public class UntipRobotAuton extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private LedSubsystem m_ledSubsystem;
  private Timer time;
  private Timer commandTime;
  private boolean isDone;

  public UntipRobotAuton(DrivetrainSubsystem drivetrainSubsystem, LedSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
    m_drivetrainSubsystem = drivetrainSubsystem;
    time = new Timer();
    commandTime = new Timer();
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandTime.start();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(commandTime.get() < 4){
      m_drivetrainSubsystem.unpitchRobot();
    } else{
      m_drivetrainSubsystem.xStance();
    }

    if((m_drivetrainSubsystem.getPitch() < unpitchDeadzone && m_drivetrainSubsystem.getPitch() > -unpitchDeadzone)){
      time.start();
      m_ledSubsystem.allGreen();
    } else{
      time.reset();
      m_ledSubsystem.allRed();
    }

    if(time.get() >= 1){
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
