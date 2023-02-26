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
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {        
        // CameraSubsystem m_cameraSubsystem;

        public static final double MAX_VOLTAGE = 11.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
        /**
        * The maximum angular velocity of the robot in radians per second.
        * <p>
        * This is a measure of how fast the robot can rotate in place.
        */
        // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        //if this is true the robot will be field oriented, and vice versa.
        public boolean isFieldOriented = true;
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

        //duh
        private AHRS gyro;
        
        private SwerveModuleState[] states;

        private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), null);

        public DrivetrainSubsystem(CameraSubsystem cameraSubsystem) {
                // m_cameraSubsystem = cameraSubsystem;
                gyro = new AHRS(SPI.Port.kMXP);

                Mk4ModuleConfiguration moduleConfig = new Mk4ModuleConfiguration();
                moduleConfig.setDriveCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
                moduleConfig.setSteerCurrentLimit(STEER_MOTOR_CURRENT_LIMIT);
                moduleConfig.setNominalVoltage(NOMINAL_DRIVE_VOLTAGE);

                m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                        // tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        FRONT_LEFT_MODULE_DRIVE_MOTOR, 
                        FRONT_LEFT_MODULE_STEER_MOTOR, 
                        FRONT_LEFT_MODULE_STEER_ENCODER, 
                        FRONT_LEFT_MODULE_STEER_OFFSET
                );

                m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                        // tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
                        FRONT_RIGHT_MODULE_STEER_MOTOR, 
                        FRONT_RIGHT_MODULE_STEER_ENCODER, 
                        FRONT_RIGHT_MODULE_STEER_OFFSET
                );
                
                m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                        // tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        BACK_LEFT_MODULE_DRIVE_MOTOR, 
                        BACK_LEFT_MODULE_STEER_MOTOR, 
                        BACK_LEFT_MODULE_STEER_ENCODER, 
                        BACK_LEFT_MODULE_STEER_OFFSET
                );

                m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                        // tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                        moduleConfig,
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        BACK_RIGHT_MODULE_DRIVE_MOTOR, 
                        BACK_RIGHT_MODULE_STEER_MOTOR, 
                        BACK_RIGHT_MODULE_STEER_ENCODER, 
                        BACK_RIGHT_MODULE_STEER_OFFSET
                );
        }
        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                gyro.zeroYaw();
        }

        public void calibrateGyro(){
                gyro.calibrate();
        }

        //returns what the drivetrain sees as gyro angle but as Rotation2d
        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(360 - gyro.getYaw());
        }

        //returns what the drivetrain sees as gyro angle but as a double
        public double getGyroscopeRotationAsDouble(){
                return 360-gyro.getYaw();
        }

        public SwerveModuleState getModuleState(SwerveModule module) {
                //return new SwerveModuleState(module.getDriveVelocity(), Rotation2d.fromDegrees(module.getSteerAngle()));
                if(module == m_frontLeftModule){
                        return states[0];
                } else if(module == m_frontRightModule){
                        return states[1];
                } else if(module == m_backLeftModule){
                        return states[2];
                } else if(module == m_backRightModule){
                        return states[3];
                } else{
                        return null;
                }
        }

        public void setState(SwerveModule module, SwerveModuleState state) {
                module.set(
                        state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        state.angle.getRadians()
                );
        }

        public void setAllStates(SwerveModuleState[] states) {
                setState(m_frontLeftModule, states[0]);
                setState(m_frontRightModule, states[1]);
                setState(m_backLeftModule, states[2]);
                setState(m_backRightModule, states[3]);
        }

        // public Pose2d getOdometry(){
        //         return m_odometry.getPoseMeters();
        // }

        // public void resetOdometry(Pose2d pose){
        //         m_odometry.resetPosition(getGyroscopeRotation(), swerveModulePositions, pose);
        // }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
                states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                setAllStates(states);
        }

        public void toggleFieldOriented(){
                isFieldOriented = !isFieldOriented;
        }

        public boolean getFieldOrientState(){
                return isFieldOriented;
        }


        @Override
        public void periodic() {
                // SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

                // m_odometry.update(getGyroscopeRotation(), swerveModulePositions);
                // SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                // m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                // m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                // m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                // m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        }
}
