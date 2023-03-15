package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
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
    frontTable = NetworkTableInstance.getDefault().getTable("limelight-frontcm");
    frontTx = frontTable.getEntry("tx");
    frontTy = frontTable.getEntry("ty");
    frontTa = frontTable.getEntry("ta");
    frontBotPose = frontTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }

  //this is run on robot init so set it to what you want here
  public void setLimelightSetting(){
    frontTable.getEntry("stream").setNumber(0);
  }

  public void createCamera(){
    // MjpegServer mjpegServer1 = new MjpegServer("frontUsbCam", 1181);
    // mjpegServer1.setSource(frontUsbCamera);
    // CameraServer.addServer("frontUsbCam");
    UsbCamera frontUsbCamera = new UsbCamera("frontUsbCamObject", 1);
    CameraServer.startAutomaticCapture(frontUsbCamera);
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

