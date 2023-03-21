package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final CommandXboxController m_controller;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               CommandXboxController controller) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_controller = controller;

        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("iscal", false);
        m_drivetrainSubsystem.calibrateGyro();
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(m_controller.rightBumper().getAsBoolean() && (m_drivetrainSubsystem.isFieldOriented || !m_drivetrainSubsystem.isFieldOriented)){
            m_drivetrainSubsystem.xStance();
        } else if(m_drivetrainSubsystem.isFieldOriented){
            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                ));
        } else if(!m_drivetrainSubsystem.isFieldOriented){
            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(), 
                    m_translationYSupplier.getAsDouble(), 
                    m_rotationSupplier.getAsDouble()
                ));
        }

        if(m_controller.rightTrigger(driveSlowDeadzone).getAsBoolean()){
            m_drivetrainSubsystem.setDriveSlow();
        } else if(!m_controller.rightTrigger(driveSlowDeadzone).getAsBoolean() && !m_drivetrainSubsystem.drivetrainState){
            m_drivetrainSubsystem.setDriveNormal();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
