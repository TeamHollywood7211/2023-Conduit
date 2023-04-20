// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GripCommand;
import frc.robot.commands.ToggleCommand;
import frc.robot.commands.TurnPurpleTimer;
import frc.robot.commands.TurnYellowTimer;
import frc.robot.commands.autons.ArmHomeAuton;
import frc.robot.commands.autons.FireFlipperAuton;
import frc.robot.commands.autons.PlaceHighAuton;
import frc.robot.commands.autons.PlaceHighShortAuton;
import frc.robot.commands.autons.UntipRobotAuton;
import frc.robot.commands.autons.XStanceAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;
import frc.robot.subsystems.TimeOfFlightSubsystem;

import static frc.robot.Constants.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> autonChooser = new SendableChooser<>();
  // The robot's controller(s)
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // The robot's subsystems
  //private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

  // public void setLimelightSetting(){
  //   m_cameraSubsystem.setLimelightSetting();
  // }

  public void createFrontUsbCamera(){
    UsbCamera frontUsbCamera = new UsbCamera("frontUsbCamObject", 1);
    CameraServer.startAutomaticCapture(frontUsbCamera);
  }

  private final CounterweightSubsystem m_counterweightSubsystem = new CounterweightSubsystem();
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final SolenoidSubsystem m_solenoidSubsystem = new SolenoidSubsystem();
  private final GripSubsystem m_gripSubsystem = new GripSubsystem();
  private final TimeOfFlightSubsystem m_timeOfFlightSubsystem = new TimeOfFlightSubsystem();
  public final LedSubsystem m_ledSubsystem = new LedSubsystem();
  
  //The robot's commands 
  private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem, m_solenoidSubsystem, m_counterweightSubsystem, m_operatorController);
  private final GripCommand m_gripCommand = new GripCommand(m_gripSubsystem, m_operatorController);
  private final ToggleCommand m_toggleCommand = new ToggleCommand(m_drivetrainSubsystem, m_driverController);
  // private final ManualCounterweightCommand m_manualCounterweightCommand = new ManualCounterweightCommand(m_counterweightSubsystem, m_driverController);
  private final DefaultDriveCommand m_driveCommand = new DefaultDriveCommand(
    m_drivetrainSubsystem, 
    () -> -modifyAxis(m_driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    m_driverController,
    m_ledSubsystem,
    m_timeOfFlightSubsystem
  );

  //dashboard sub
  private final DashboardSubsystem m_DashboardSubsystem = new DashboardSubsystem(m_armSubsystem, m_counterweightSubsystem, m_drivetrainSubsystem, m_solenoidSubsystem, m_gripSubsystem, m_timeOfFlightSubsystem);
  //the robot's auton parts
  private FireFlipperAuton m_fireFlipperAuton = new FireFlipperAuton(m_solenoidSubsystem);
  // private ArmToLowAuton m_armToLowAuton = new ArmToLowAuton(m_armSubsystem, m_solenoidSubsystem);
  private ArmHomeAuton m_armHomeAuton = new ArmHomeAuton(m_armSubsystem, m_solenoidSubsystem);
  private PlaceHighAuton m_placeHighAuton = new PlaceHighAuton(m_solenoidSubsystem, m_gripSubsystem, m_armSubsystem);
  private PlaceHighShortAuton m_placeHighShortAuton = new PlaceHighShortAuton(m_solenoidSubsystem, m_gripSubsystem, m_armSubsystem);
  private UntipRobotAuton m_untipRobotAuton = new UntipRobotAuton(m_drivetrainSubsystem, m_ledSubsystem);
  private XStanceAuton m_xStanceAuton = new XStanceAuton(m_drivetrainSubsystem);
  private TurnPurpleTimer turnPurpleTimer = new TurnPurpleTimer(m_ledSubsystem);
  private TurnYellowTimer turnYellowTimer = new TurnYellowTimer(m_ledSubsystem);

  public HashMap<String, Command> eventMap = new HashMap<>(Map.ofEntries(
    Map.entry("firesol", m_fireFlipperAuton),
    Map.entry("grabcube", new InstantCommand(m_gripSubsystem::setGripCube, m_gripSubsystem)),
    Map.entry("grabcone", new InstantCommand(m_gripSubsystem::setGripCone, m_gripSubsystem)),
    Map.entry("grabconeloose", new InstantCommand(m_gripSubsystem::setGripConeLoose)),
    Map.entry("grabout", new InstantCommand(m_gripSubsystem::setGripOut, m_gripSubsystem)),
    Map.entry("extendwrist", new InstantCommand(m_solenoidSubsystem::extendWrist, m_solenoidSubsystem)),
    Map.entry("retractwrist", new InstantCommand(m_solenoidSubsystem::retractWrist, m_solenoidSubsystem)),
    Map.entry("extendarm", new InstantCommand(m_solenoidSubsystem::extendArm, m_solenoidSubsystem)),
    Map.entry("retractarm", new InstantCommand(m_solenoidSubsystem::retractArm, m_solenoidSubsystem)),
    Map.entry("armhigh", new InstantCommand(m_armSubsystem::setArmHigh, m_armSubsystem)),
    Map.entry("armmid", new InstantCommand(m_armSubsystem::setArmMid, m_armSubsystem)),
    Map.entry("armlow", new InstantCommand(m_armSubsystem::setArmLow)),
    Map.entry("armlowest", new InstantCommand(m_armSubsystem::setArmJustBelowLow)),
    Map.entry("armin", new InstantCommand(m_armSubsystem::setArmStored, m_armSubsystem)),
    Map.entry("armwristhome", m_armHomeAuton),
    Map.entry("wait1sec", new WaitCommand(1)),
    Map.entry("wait0.75sec", new WaitCommand(0.75)),
    Map.entry("wait0.5sec", new WaitCommand(0.5)),
    Map.entry("armslightup", new InstantCommand(m_armSubsystem::setArmJustAboveLow, m_armSubsystem)),
    Map.entry("placehigh", m_placeHighAuton),
    Map.entry("placehighshort", m_placeHighShortAuton),
    Map.entry("zerogyro", new InstantCommand(m_drivetrainSubsystem::zeroGyroscope)),
    Map.entry("unpitch", m_untipRobotAuton),
    Map.entry("xstance", m_xStanceAuton),
    Map.entry("print", new PrintCommand("===========================didthething==================================="))
  ));;

  //robot trajectories
  final List<PathPlannerTrajectory> throwAndPark = PathPlanner.loadPathGroup("Over and Back", new PathConstraints(1, 2));
  final List<PathPlannerTrajectory> driveGrabPlace = PathPlanner.loadPathGroup("Drive Grab Place", new PathConstraints(2, 2), new PathConstraints(2, 2));
  final PathPlannerTrajectory park = PathPlanner.loadPath("Park", new PathConstraints(1, 1));
  final List<PathPlannerTrajectory> bumpSide = PathPlanner.loadPathGroup("Bump Side", new PathConstraints(2, 1.5));
  final List<PathPlannerTrajectory> placeTwoHigh = PathPlanner.loadPathGroup("Place Two High", new PathConstraints(3, 3), new PathConstraints(3, 3), new PathConstraints(1, 1));
  final List<PathPlannerTrajectory> dukesOfHazard = PathPlanner.loadPathGroup("Dukes of Hazard", new PathConstraints(3.4, 3.25), new PathConstraints(2, 2));
  final List<PathPlannerTrajectory> oneHighConeAndPark = PathPlanner.loadPathGroup("Grab Cone and Park", new PathConstraints(1.3, 1.8), new PathConstraints(2, 2.5));
  final List<PathPlannerTrajectory> placeCubeGrabConePark = PathPlanner.loadPathGroup("Place Cube Grab Cone Park",new PathConstraints(2, 2));
  //Auto builder, use this to turn trajectories into actual paths
  SwerveAutoBuilder stateAutoBuilder = new SwerveAutoBuilder(
    m_drivetrainSubsystem::getPose2d, 
    m_drivetrainSubsystem::resetPose2d, 
    m_drivetrainSubsystem.getKinematics(), 
    new PIDConstants(AUTON_TRANSLATE_P, 0, 0),
    new PIDConstants(AUTON_ROTATE_P, 0, 0), 
    m_drivetrainSubsystem::setAllStates, 
    eventMap, 
    true,
    m_drivetrainSubsystem
  );


  private Command throwAndParkCommand = stateAutoBuilder.fullAuto(throwAndPark);
  private Command driveGrabPlaceCommand = stateAutoBuilder.fullAuto(driveGrabPlace);
  private Command parkCommand = stateAutoBuilder.fullAuto(park);
  private Command bumpSideCommand = stateAutoBuilder.fullAuto(bumpSide);
  private Command placeTwoHighCommand = stateAutoBuilder.fullAuto(placeTwoHigh);
  private Command dukesOfHazardCommand = stateAutoBuilder.fullAuto(dukesOfHazard);
  private Command grabConeAndParkCommand = stateAutoBuilder.fullAuto(oneHighConeAndPark);
  private Command placeCubeGrabConeParkCommand = stateAutoBuilder.fullAuto(placeCubeGrabConePark);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    m_solenoidSubsystem.enableAnalogCompressor();

    configureAutons();
    m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);
    // Configure the button bindings
    configureButtonBindings();
    configureAllMotors();
  }

  public void configureAutons(){
    // eventMap.put("firesol", m_fireFlipperAuton);
    // eventMap.put("grabcube", m_grabCubeAuton);
    // eventMap.put("armlow", m_armToLowAuton);

    autonChooser.setDefaultOption("Do nothing", new InstantCommand());
    autonChooser.addOption("Over and Park", throwAndParkCommand);
    autonChooser.addOption("Drive Grab Place", driveGrabPlaceCommand);
    autonChooser.addOption("Park on Table", parkCommand);
    autonChooser.addOption("Bump Side", bumpSideCommand);
    autonChooser.addOption("Place Two High", placeTwoHighCommand);
    autonChooser.addOption("Dukes of Hazard", dukesOfHazardCommand);
    autonChooser.addOption("Place Cone Park", grabConeAndParkCommand);
    autonChooser.addOption("Place Cube Park", placeCubeGrabConeParkCommand);
    SmartDashboard.putData(autonChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Trigger(m_driverController.back())
      .onTrue(m_toggleCommand);

    // //when pressed, the start button should toggle field orientation on and off
    new Trigger(m_driverController.start())
      .onTrue(m_toggleCommand);

    //Buttons set the lights to either yellow or purple
    new Trigger(m_driverController.povLeft())
      .onTrue(turnYellowTimer);

    new Trigger(m_driverController.povRight())
      .onTrue(turnPurpleTimer);

    new Trigger(m_driverController.povUp())
      .onTrue(new InstantCommand(m_ledSubsystem::enabledAnim, m_ledSubsystem));

    new Trigger(m_driverController.povDown())
      .onTrue(new InstantCommand(m_ledSubsystem::allOff, m_ledSubsystem));

    
    /*
    these three buttons do the same thing because in the command it checks to see which button is pressed 
    to do a thing. These buttons set the arm to different positions: Y-high X-mid A-low
    */
    new Trigger(m_operatorController.y())
      .onTrue(m_armCommand);
    new Trigger(m_operatorController.x())
      .onTrue(m_armCommand);
    new Trigger(m_operatorController.a())
      .onTrue(m_armCommand);
    new Trigger(m_operatorController.b())
      .onTrue(m_armCommand);

    new Trigger(m_operatorController.leftStick())
      .onTrue(m_armCommand);

    //led control for op
    new Trigger(m_operatorController.povRight())
      .onTrue(new InstantCommand(m_ledSubsystem::allPurple));
    new Trigger(m_operatorController.povLeft())
      .onTrue(new InstantCommand(m_ledSubsystem::allYellow));
    new Trigger(m_operatorController.povUp())
      .onTrue(new InstantCommand(m_ledSubsystem::enabledAnim, m_ledSubsystem));
    new Trigger(m_operatorController.povDown())
      .onTrue(new InstantCommand(m_ledSubsystem::allRainbow, m_ledSubsystem));
      //.onTrue(new InstantCommand(this::printComments));


    //left trigger toggles the wrist solenoid
    new Trigger(m_operatorController.button(5))
      .onTrue(m_armCommand);

    //right trigger grips carefully
    new Trigger(m_operatorController.button(6))
      .onTrue(m_gripCommand);

    //creates the triggers on the operator controller to make them open and close the gripper
    new Trigger(m_operatorController.rightTrigger(0.1))
      .onTrue(m_gripCommand);
    new Trigger(m_operatorController.leftTrigger(0.1))
      .onTrue(m_gripCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private void configureAllMotors(){
    m_armSubsystem.configureMotorControllers();
    m_gripSubsystem.configureMotorControllers();
    m_counterweightSubsystem.configureCounterweightMotor();
  }
}
