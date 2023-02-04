// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GripCommand;
import frc.robot.commands.InitializeCommand;
import frc.robot.commands.ToggleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's controller(s)
  //private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController m_controller = new CommandXboxController(0);
  // The robot's subsystems
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final SolenoidSubsystem m_solenoidSubsystem = new SolenoidSubsystem();
  private final CounterweightSubsystem m_counterweightSubsystem = new CounterweightSubsystem();
  
  //The robot's commands 
  private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem, m_solenoidSubsystem, m_counterweightSubsystem, m_controller);
  public final InitializeCommand m_InitializeCommand = new InitializeCommand(m_armSubsystem, m_counterweightSubsystem, m_solenoidSubsystem);
  private final GripCommand m_gripCommand = new GripCommand(m_armSubsystem, m_controller);
  private final ToggleCommand m_toggleCommand = new ToggleCommand(m_drivetrainSubsystem, m_controller);
  private final DefaultDriveCommand m_driveCommand = new DefaultDriveCommand(
    m_drivetrainSubsystem, 
    () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
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
    new Trigger(m_controller.back())
      .onTrue(m_toggleCommand);
    
    
    /*
    these three buttons do the same thing because in the command it checks to see which button is pressed 
    to do a thing. These buttons set the arm to different positions: Y-high X-mid A-low
    */
    new Trigger(m_controller.y())
      .onTrue(m_armCommand);
    new Trigger(m_controller.x())
      .onTrue(m_armCommand);
    new Trigger(m_controller.a())
      .onTrue(m_armCommand);
    new Trigger(m_controller.b())
      .onTrue(m_armCommand);

    new Trigger(m_controller.rightTrigger(0.1))
      .onTrue(m_gripCommand);
    new Trigger(m_controller.leftTrigger(0.1))
      .onTrue(m_gripCommand);
    
    // //when pressed, the start button should toggle field orientation on and off
    new Trigger(m_controller.start())
      .onTrue(m_toggleCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
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
