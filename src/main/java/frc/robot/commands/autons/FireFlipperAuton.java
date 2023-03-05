package frc.robot.commands.autons;

import com.kauailabs.navx.frc.Tracer;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SolenoidSubsystem;

public class FireFlipperAuton extends CommandBase {
  SolenoidSubsystem m_solenoidSubsystem;
  Boolean isDone;
  Timer time;

  public FireFlipperAuton(SolenoidSubsystem solenoidSubsystem) {
    m_solenoidSubsystem = solenoidSubsystem;
    time = new Timer();
    addRequirements(solenoidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Commands.print("10101010101010101101010101101010101101010101101010101101010101");
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_solenoidSubsystem.fireFlipperSolenoid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_solenoidSubsystem.disableFlipperSolenoid();
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_solenoidSubsystem.getFlipperSolenoidState() && time.get()>0.5){
      return true;
    }
    return false;
  }
}
