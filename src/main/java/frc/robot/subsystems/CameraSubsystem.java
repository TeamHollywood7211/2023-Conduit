package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  private NetworkTable frontTable;
  private NetworkTableEntry frontTx;
  private NetworkTableEntry frontTy;
  private NetworkTableEntry frontTa;
  private double frontX;
  private double frontY;
  private double frontA;
  public double[] frontBotPose;

  private NetworkTable backTable;

  public CameraSubsystem() {
    frontTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    frontTx = frontTable.getEntry("tx");
    frontTy = frontTable.getEntry("ty");
    frontTa = frontTable.getEntry("ta");
    frontBotPose = frontTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    frontTable.getEntry("stream").setNumber(2);
  }

  @Override
  public void periodic() {
    frontX = frontTx.getDouble(0);
    frontY = frontTy.getDouble(0);
    frontA = frontTa.getDouble(0);
    frontBotPose = frontTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    SmartDashboard.putNumberArray("frontbotpose", frontBotPose);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

