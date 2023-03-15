package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class DashboardSubsystem extends SubsystemBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ArmSubsystem m_armSubsystem;
  private CounterweightSubsystem m_counterweightSubsystem;
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private SolenoidSubsystem m_solenoidSubsystem;
  private GripSubsystem m_gripSubsystem;
  private final Field2d m_field = new Field2d();

  public DashboardSubsystem(ArmSubsystem armSubsystem, CounterweightSubsystem counterweightSubsystem, DrivetrainSubsystem drivetrainSubsystem, SolenoidSubsystem solenoidSubsystem, GripSubsystem gripSubsystem, SendableChooser autonChooser) {
    m_armSubsystem = armSubsystem;
    m_counterweightSubsystem = counterweightSubsystem;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_solenoidSubsystem = solenoidSubsystem;
    m_gripSubsystem = gripSubsystem;

    // ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTab");
    // driveTab.addCamera("FrontCam", "front-limelight", null);
    // driveTab.add(autonChooser);
    // SmartDashboard.putData("Field", m_field);
    // SmartDashboard.putNumber("arm P", armkP);
    // SmartDashboard.putNumber("arm I", armkI);
    // SmartDashboard.putNumber("arm D", armkD);
  }

  public void periodic(){
    //ARM DASHBOARD STUFF
    SmartDashboard.putNumber("ARM POS", m_armSubsystem.getArmPos());
    // SmartDashboard.putBoolean("is outside frame", m_armSubsystem.armOutsideFramePerim(0));
    // SmartDashboard.putNumber("Arm Current", m_armSubsystem.getArmCurrent());

    //GRIP DASHBOARD STUFF
    SmartDashboard.putNumber("Grip Current", m_gripSubsystem.getGripCurrent());
    SmartDashboard.putNumber("Grip Pose", m_gripSubsystem.getGripPos());
    SmartDashboard.putNumber("Grip Temp Farenheit", m_gripSubsystem.getGripTemp()*(9/5)+32);

    //COUNTERWEIGHT DASHBOARD STUFF
    // SmartDashboard.putNumber("Counterweight Motor Position", m_counterweightSubsystem.getCounterweightPos());
    // SmartDashboard.putNumber("Counterweight Current", m_counterweightSubsystem.getCounterweightCurrent());

    //SOLENOID DASHBOARD STUFF
    // SmartDashboard.putBoolean("wrist solenoid state", m_solenoidSubsystem.getWristSolenoidState());
    // SmartDashboard.putBoolean("armSolenoid State", m_solenoidSubsystem.getArmSolenoidState());

    //DRIVETRAIN DASHBOARD STUFF
    SmartDashboard.putNumber("Gyroscope Position as Double", m_drivetrainSubsystem.getGyroscopeRotationAsDouble());
    SmartDashboard.putBoolean("Field Orientation", m_drivetrainSubsystem.getFieldOrientState());

    //FIELD DASHBOARD STUFF
    // m_field.setRobotPose(m_drivetrainSubsystem.getPose2d());

    //SOLENOID STUFF
    SmartDashboard.putNumber("PSI", m_solenoidSubsystem.getCompressorPSI());
  }

  // Returns true when the command should end.
}
