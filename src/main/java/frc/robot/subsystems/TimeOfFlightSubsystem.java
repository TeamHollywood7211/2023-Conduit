package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSubsystem extends SubsystemBase {
  private TimeOfFlight tof;
  public TimeOfFlightSubsystem() {
    tof = new TimeOfFlight(1);
    tof.setRangingMode(RangingMode.Medium, 580);
  }

  public double getDistance(){
    return tof.getRange();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
