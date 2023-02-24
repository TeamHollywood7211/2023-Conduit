// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

        public SwerveDriveOdometry m_odometry;
        
        SwerveModuleState[] states;
        SwerveModulePosition[] swerveModulePositions;
        SwerveModulePosition frontLeftPose;
        SwerveModulePosition frontRightPose;
        SwerveModulePosition backLeftPose;
        SwerveModulePosition backRightPose;

        CameraSubsystem m_cameraSubsystem;

        public static final double MAX_VOLTAGE = 12.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
        /**
        * The maximum angular velocity of the robot in radians per second.
        * <p>
        * This is a measure of how fast the robot can rotate in place.
        */
        // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private AHRS gyro;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public boolean isFieldOriented = true;
        
        public DrivetrainSubsystem(CameraSubsystem cameraSubsystem) {
                m_cameraSubsystem = cameraSubsystem;
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                gyro = new AHRS(SPI.Port.kMXP);

                Mk4ModuleConfiguration moduleConfig = new Mk4ModuleConfiguration();
                moduleConfig.setDriveCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
                moduleConfig.setSteerCurrentLimit(STEER_MOTOR_CURRENT_LIMIT);
                moduleConfig.setNominalVoltage(NOMINAL_DRIVE_VOLTAGE);

                m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        FRONT_LEFT_MODULE_DRIVE_MOTOR, 
                        FRONT_LEFT_MODULE_STEER_MOTOR, 
                        FRONT_LEFT_MODULE_STEER_ENCODER, 
                        FRONT_LEFT_MODULE_STEER_OFFSET
                );

                m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
                        FRONT_RIGHT_MODULE_STEER_MOTOR, 
                        FRONT_RIGHT_MODULE_STEER_ENCODER, 
                        FRONT_RIGHT_MODULE_STEER_OFFSET
                );
                
                m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        BACK_LEFT_MODULE_DRIVE_MOTOR, 
                        BACK_LEFT_MODULE_STEER_MOTOR, 
                        BACK_LEFT_MODULE_STEER_ENCODER, 
                        BACK_LEFT_MODULE_STEER_OFFSET
                );

                m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        BACK_RIGHT_MODULE_DRIVE_MOTOR, 
                        BACK_RIGHT_MODULE_STEER_MOTOR, 
                        BACK_RIGHT_MODULE_STEER_ENCODER, 
                        BACK_RIGHT_MODULE_STEER_OFFSET
                );

                states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                frontLeftPose = new SwerveModulePosition(states[0].speedMetersPerSecond, states[0].angle);
                frontRightPose = new SwerveModulePosition(states[1].speedMetersPerSecond, states[1].angle);
                backLeftPose = new SwerveModulePosition(states[2].speedMetersPerSecond, states[2].angle);
                backRightPose = new SwerveModulePosition(states[3].speedMetersPerSecond, states[3].angle);
                swerveModulePositions = new SwerveModulePosition[] {
                        new SwerveModulePosition(states[0].speedMetersPerSecond, states[0].angle),
                        new SwerveModulePosition(states[1].speedMetersPerSecond, states[1].angle),
                        new SwerveModulePosition(states[2].speedMetersPerSecond, states[2].angle),
                        new SwerveModulePosition(states[3].speedMetersPerSecond, states[3].angle)
                };

                m_odometry = new SwerveDriveOdometry(
                        m_kinematics, 
                        getGyroscopeRotation(), 
                        swerveModulePositions
                );
        }
        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                // FIXME Remove if you are using a Pigeon
                gyro.zeroYaw();
                // FIXME Uncomment if you are using a NavX
        }

        public void calibrateGyro(){
                gyro.calibrate();
                System.out.print("calibrating");
                if(gyro.isCalibrating()){
                        System.out.print("...");
                }
        }

        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(360 - gyro.getYaw());
        }

        public Pose2d getOdometry(){
                return m_odometry.getPoseMeters();
        }

        public void resetOdometry(Pose2d pose){
                m_odometry.resetPosition(getGyroscopeRotation(), swerveModulePositions, pose);
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        public void toggleFieldOriented(){
                isFieldOriented = !isFieldOriented;
        }


        @Override
        public void periodic() {
                SmartDashboard.putNumber("Gyroscope Position from Swerve", gyro.getYaw());
                SmartDashboard.putBoolean("fieldoriented", isFieldOriented);
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                swerveModulePositions = new SwerveModulePosition[]{
                        new SwerveModulePosition(states[0].speedMetersPerSecond, states[0].angle),
                        new SwerveModulePosition(states[1].speedMetersPerSecond, states[1].angle),
                        new SwerveModulePosition(states[2].speedMetersPerSecond, states[2].angle),
                        new SwerveModulePosition(states[3].speedMetersPerSecond, states[3].angle)
                };

                m_odometry.update(getGyroscopeRotation(), swerveModulePositions);
                //SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                SwerveDriveKinematics.desaturateWheelSpeeds(states,MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        }
}
