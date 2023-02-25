// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FireFlipperAuton;
import frc.robot.commands.GripCommand;
import frc.robot.commands.InitializeCommand;
import frc.robot.commands.ManualCounterweightCommand;
import frc.robot.commands.ToggleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> autonChooser = new SendableChooser<>();
  // The robot's controller(s)
  //private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  // The robot's subsystems
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_cameraSubsystem);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final SolenoidSubsystem m_solenoidSubsystem = new SolenoidSubsystem();
  private final CounterweightSubsystem m_counterweightSubsystem = new CounterweightSubsystem();
  
  //The robot's commands 
  private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem, m_solenoidSubsystem, m_counterweightSubsystem, m_operatorController);
  public final InitializeCommand m_InitializeCommand = new InitializeCommand(m_armSubsystem, m_counterweightSubsystem, m_operatorController);
  private final GripCommand m_gripCommand = new GripCommand(m_armSubsystem, m_operatorController);
  private final ToggleCommand m_toggleCommand = new ToggleCommand(m_drivetrainSubsystem, m_driverController);
  private final ManualCounterweightCommand m_manualCounterweightCommand = new ManualCounterweightCommand(m_counterweightSubsystem, m_driverController);
  private final DefaultDriveCommand m_driveCommand = new DefaultDriveCommand(
    m_drivetrainSubsystem, 
    () -> -modifyAxis(m_driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
  );

  private FireFlipperAuton m_fireFlipperAuton = new FireFlipperAuton(m_solenoidSubsystem);

  // public SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  //   m_drivetrainSubsystem::getOdometry, 
  //   m_drivetrainSubsystem::resetOdometry, 
  //   new PIDConstants(1.0, 0.0, 0.1), 
  //   new PIDConstants(1.0, 0.0, 0.1), 
  //   m_drivetrainSubsystem::drive, 
  //   autonEventMap,
  //   true,
  //   m_drivetrainSubsystem
  // );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    autonChooser.setDefaultOption("Do nothing", new InstantCommand());
    autonChooser.addOption("Fire Cylinder", m_fireFlipperAuton);
    SmartDashboard.putData(autonChooser);
    m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);

    // Configure the button bindings
    configureButtonBindings();
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
  }

  public void configureAutons(){
    autonEventMap.put("fireFlipperSolenoid", m_fireFlipperAuton);
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
