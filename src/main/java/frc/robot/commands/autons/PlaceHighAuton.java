package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;
import static frc.robot.Constants.*;

public class PlaceHighAuton extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SolenoidSubsystem m_solenoidSubsystem;
  private GripSubsystem m_gripSubsystem;
  private ArmSubsystem m_armSubsystem;
  Timer time;

  public PlaceHighAuton(SolenoidSubsystem solenoidSubsystem, GripSubsystem gripSubsystem, ArmSubsystem armSubsystem){
    m_armSubsystem = armSubsystem;
    m_solenoidSubsystem = solenoidSubsystem;
    m_gripSubsystem = gripSubsystem;
    time = new Timer();
    addRequirements(solenoidSubsystem, gripSubsystem, armSubsystem);
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
    if(time.get()>0.5){
      m_armSubsystem.setArmHigh();
    }
    if(m_armSubsystem.getArmPos()>armHighTarget-10){
      m_solenoidSubsystem.extendArm();
    }
    if(time.get()>2){
      m_solenoidSubsystem.extendWrist();
    }
    if(time.get()>2.5){
      m_gripSubsystem.setGripOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time.get() > 2.75){
      return true;
    }
    return false;
  }
}
