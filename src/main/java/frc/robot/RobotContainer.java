// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import frc.robot.commands.InitializeCommand;
import frc.robot.commands.ToggleCommand;
import frc.robot.commands.LedCommand;
import frc.robot.commands.autons.ArmHomeAuton;
import frc.robot.commands.autons.ArmToLowAuton;
import frc.robot.commands.autons.FireFlipperAuton;
import frc.robot.commands.autons.GrabCubeAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;
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
  private final CommandXboxController m_signalController = new CommandXboxController(2);

  // The robot's subsystems
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_cameraSubsystem);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();
  private final SolenoidSubsystem m_solenoidSubsystem = new SolenoidSubsystem();
  private final CounterweightSubsystem m_counterweightSubsystem = new CounterweightSubsystem();
  private final GripSubsystem m_gripSubsystem = new GripSubsystem();
  
  //The robot's commands 
  private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem, m_solenoidSubsystem, m_counterweightSubsystem, m_operatorController);
  public final InitializeCommand m_InitializeCommand = new InitializeCommand(m_armSubsystem, m_counterweightSubsystem, m_operatorController);
  private final GripCommand m_gripCommand = new GripCommand(m_gripSubsystem, m_operatorController);
  private final ToggleCommand m_toggleCommand = new ToggleCommand(m_drivetrainSubsystem, m_driverController);
  private final LedCommand m_LedCommand = new LedCommand(m_LedSubsystem, m_signalController);

  // private final ManualCounterweightCommand m_manualCounterweightCommand = new ManualCounterweightCommand(m_counterweightSubsystem, m_driverController);
  private final DefaultDriveCommand m_driveCommand = new DefaultDriveCommand(
    m_drivetrainSubsystem, 
    () -> -modifyAxis(m_driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
  );

  //dashboard sub
  private final DashboardSubsystem m_DashboardSubsystem = new DashboardSubsystem(m_armSubsystem, m_counterweightSubsystem, m_drivetrainSubsystem, m_solenoidSubsystem, m_gripSubsystem, autonChooser);
  //the robot's auton parts
  private FireFlipperAuton m_fireFlipperAuton = new FireFlipperAuton(m_solenoidSubsystem);
  private ArmToLowAuton m_armToLowAuton = new ArmToLowAuton(m_armSubsystem, m_solenoidSubsystem);
  private GrabCubeAuton m_grabCubeAuton = new GrabCubeAuton(m_solenoidSubsystem, m_gripSubsystem);
  private ArmHomeAuton m_armHomeAuton = new ArmHomeAuton(m_armSubsystem, m_solenoidSubsystem);

  public HashMap<String, Command> eventMap = new HashMap<>(Map.ofEntries(
    Map.entry("firesol", m_fireFlipperAuton),
    Map.entry("grabcube", m_grabCubeAuton),
    Map.entry("grabcone", new InstantCommand(m_gripSubsystem::setGripCone, m_gripSubsystem)),
    Map.entry("grabout", new InstantCommand(m_gripSubsystem::setGripOut, m_gripSubsystem)),
    Map.entry("wristout", new InstantCommand(m_solenoidSubsystem::extendWrist, m_solenoidSubsystem)),
    Map.entry("wristin", new InstantCommand(m_solenoidSubsystem::retractWrist, m_solenoidSubsystem)),
    Map.entry("armlow", m_armToLowAuton),
    Map.entry("armin", new InstantCommand(m_armSubsystem::setArmStored, m_armSubsystem)),
    Map.entry("armwristhome", m_armHomeAuton),
    Map.entry("wait1sec", new WaitCommand(1)),
    Map.entry("wait0.75sec", new WaitCommand(0.75)),
    Map.entry("wait0.5sec", new WaitCommand(0.5)),
    Map.entry("armslightup", new InstantCommand(m_armSubsystem::setArmJustAboveLow, m_armSubsystem)),
    Map.entry("print", new PrintCommand("===========================didthething==================================="))
  ));;

  //robot trajectories
  final List<PathPlannerTrajectory> throwAndPark = PathPlanner.loadPathGroup("Flip, Over and Back", new PathConstraints(1, 0.5));
  final List<PathPlannerTrajectory> driveGrabPlace = PathPlanner.loadPathGroup("Drive Grab Place", new PathConstraints(2, 2), new PathConstraints(2, 2));
  final PathPlannerTrajectory park = PathPlanner.loadPath("Park", new PathConstraints(1, 1));
  final List<PathPlannerTrajectory> bumpSide = PathPlanner.loadPathGroup("Bump Side", new PathConstraints(1, 1));

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
  private Command driveGrabParkCommand = stateAutoBuilder.fullAuto(driveGrabPlace);
  private Command parkCommand = stateAutoBuilder.fullAuto(park);
  private Command bumpSideCommand = stateAutoBuilder.fullAuto(bumpSide);

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
  }

  public void configureAutons(){
    // eventMap.put("firesol", m_fireFlipperAuton);
    // eventMap.put("grabcube", m_grabCubeAuton);
    // eventMap.put("armlow", m_armToLowAuton);

    autonChooser.setDefaultOption("Do nothing", new InstantCommand());
    autonChooser.addOption("Fire Cylinder", m_fireFlipperAuton);
    autonChooser.addOption("Flip, Over and Back", throwAndParkCommand);
    autonChooser.addOption("Drive Grab Place", driveGrabParkCommand);
    autonChooser.addOption("Park on Table", parkCommand);
    autonChooser.addOption("Bump Side", bumpSideCommand);
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

    //triggers that allow for manual control of the counterweight, mostly for testing of center of grav
    // new Trigger(m_driverController.povDown())
    //   .whileTrue(m_manualCounterweightCommand);
    // new Trigger(m_driverController.povUp())
    //   .whileTrue(m_manualCounterweightCommand);
  
    
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

    //back button rezeros the arm subsystem and the counterweight subsystem
    // new Trigger(m_operatorController.back())
    //   .onTrue(m_InitializeCommand);

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

    // TODO: Katona - I'm 99% sure these aren't needed.  
    //       button 5 works, but isn't listed here.  (See LedCommand.java)
    //       Leaving for 1 check-in to verify behavior.
    new Trigger(m_signalController.button(1))
       .onTrue(m_LedCommand);
    new Trigger(m_signalController.button(2))
       .onTrue(m_LedCommand);
    new Trigger(m_signalController.button(3))
       .onTrue(m_LedCommand);
    new Trigger(m_signalController.button(4))
       .onTrue(m_LedCommand);   
    new Trigger(m_signalController.button(5))
       .onTrue(m_LedCommand);
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
}
