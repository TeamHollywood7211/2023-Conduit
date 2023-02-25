package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardSubsystem extends SubsystemBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ArmSubsystem m_armSubsystem;
  private CounterweightSubsystem m_counterweightSubsystem;
  //private DrivetrainSubsystem m_drivetrainSubsystem;
  private SolenoidSubsystem m_solenoidSubsystem;

  public DashboardSubsystem(ArmSubsystem armSubsystem, CounterweightSubsystem counterweightSubsystem, DrivetrainSubsystem drivetrainSubsystem, SolenoidSubsystem solenoidSubsystem) {
    m_armSubsystem = armSubsystem;
    m_counterweightSubsystem = counterweightSubsystem;
    //m_drivetrainSubsystem = drivetrainSubsystem;
    m_solenoidSubsystem = solenoidSubsystem;
  }

  public void periodic(){
    //ARM DASHBOARD STUFF
    SmartDashboard.putNumber("ARM POS", m_armSubsystem.getAnglePos());
    SmartDashboard.putBoolean("is outside frame", m_armSubsystem.armOutsideFramePerim(0));
    SmartDashboard.putNumber("Arm Current", m_armSubsystem.getArmCurrent());

    //GRIP DASHBOARD STUFF
    SmartDashboard.putNumber("Grip Current", m_armSubsystem.getGripCurrent());

    //COUNTERWEIGHT DASHBOARD STUFF
    SmartDashboard.putNumber("Counterweight Motor Position", m_counterweightSubsystem.getCounterweightPos());
    SmartDashboard.putNumber("Counterweight Current", m_counterweightSubsystem.getCounterweightCurrent());

    //SOLENOID DASHBOARD STUFF
    SmartDashboard.putBoolean("wrist solenoid state", m_solenoidSubsystem.getWristSolenoidState());
    SmartDashboard.putBoolean("armSolenoid State", m_solenoidSubsystem.getArmSolenoidState());
  }

  // Returns true when the command should end.
}
