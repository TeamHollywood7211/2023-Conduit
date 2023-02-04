// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.SolenoidSubsystem;

/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_armSubsystem;
    private final CommandXboxController m_controller;
    private final SolenoidSubsystem m_solenoidSubsystem;
    private final CounterweightSubsystem m_counterweightSubsystem;

    private boolean isDone;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmCommand(ArmSubsystem armSubsystem, SolenoidSubsystem solenoidSubsystem, CounterweightSubsystem counterweightSubsystem, CommandXboxController controller) {
        m_solenoidSubsystem = solenoidSubsystem;
        m_armSubsystem = armSubsystem;
        m_counterweightSubsystem = counterweightSubsystem;
        m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isDone = false;
        //m_solenoidSubsystem.retractArm();
        //m_solenoidSubsystem.retractWrist();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_controller.y().getAsBoolean()){
            if(!m_solenoidSubsystem.getWristSolenoidState() && m_armSubsystem.armOutsideFramePerim(0)){
                m_solenoidSubsystem.extendWrist();
            }
            if(m_armSubsystem.armOutsideFramePerim(20)){
                m_solenoidSubsystem.extendArm();
            }
            m_counterweightSubsystem.setCounterweightHigh();
            m_armSubsystem.setArmHigh();
            if(m_solenoidSubsystem.getWristSolenoidState()){
                isDone = true;
            }
        }
        if(m_controller.x().getAsBoolean()){
            if(m_solenoidSubsystem.getArmSolenoidState()){
                m_solenoidSubsystem.retractArm();
            }
            if(!m_solenoidSubsystem.getWristSolenoidState() && m_armSubsystem.armOutsideFramePerim(0)){
                m_solenoidSubsystem.extendWrist();
            }
            m_counterweightSubsystem.setCounterweightMid();
            m_armSubsystem.setArmMid();
            if(m_solenoidSubsystem.getWristSolenoidState()){
                isDone = true;
            }
        }
        if(m_controller.a().getAsBoolean()){
            if(m_solenoidSubsystem.getArmSolenoidState()){
                m_solenoidSubsystem.retractArm();
            }
            if(!m_solenoidSubsystem.getWristSolenoidState() && m_armSubsystem.armOutsideFramePerim(-4)){
                m_solenoidSubsystem.extendWrist();
            }
            m_counterweightSubsystem.setCounterweightLow();
            m_armSubsystem.setArmLow();
            if(m_solenoidSubsystem.getWristSolenoidState()){
                isDone = true;
            }
        }
        if(m_controller.b().getAsBoolean()){
            if(m_solenoidSubsystem.getArmSolenoidState()){
                m_solenoidSubsystem.retractArm();
            }
            m_solenoidSubsystem.retractWrist();
            m_counterweightSubsystem.setCounterweightStored();
            m_armSubsystem.setArmStored();
            isDone = true;
        }
        SmartDashboard.putBoolean("ran arm", true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(isDone){
            return true;
        }
        return false;
    }
}
