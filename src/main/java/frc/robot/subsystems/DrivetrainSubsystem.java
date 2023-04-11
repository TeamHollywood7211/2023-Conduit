// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {        
        // CameraSubsystem m_cameraSubsystem;
        CounterweightSubsystem m_counterweightSubsystem;
        //this is true when normal speeds false when slow speeds
        public boolean drivetrainState = true;

        /**
        * The maximum angular velocity of the robot in radians per second.
        * <p>
        * This is a measure of how fast the robot can rotate in place.
        */
        // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.

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
        private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
        
        private SwerveModuleState[] states;

        private SwerveDriveOdometry m_odometry;

        private SwerveModulePosition[] swerveModulePositions;

        private PIDController unpitchPIDController;

        // private double lastPitch;

        private Timer everySecondTimer;
        

        public DrivetrainSubsystem(CounterweightSubsystem counterweightSubsystem) {
                m_counterweightSubsystem = counterweightSubsystem;
                MkModuleConfiguration moduleConfig = new MkModuleConfiguration();
                moduleConfig.setDriveCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
                moduleConfig.setSteerCurrentLimit(STEER_MOTOR_CURRENT_LIMIT);
                moduleConfig.setNominalVoltage(NOMINAL_DRIVE_VOLTAGE);
                moduleConfig.setSteerPID(1.0, 0.0, 0.1);

                unpitchPIDController = new PIDController(unpitchkP, unpitchkI, unpitchkD);
                unpitchPIDController.setTolerance(unpitchTolerance);

                ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

                m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                        .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                        .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
                        .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                        .build();
                
                m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
                        .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                        .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
                        .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                        .build();
                
                m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                        .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                        .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
                        .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                        .build();
                
                m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
                        .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0))
                        .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                        .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
                        .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
                        .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
                        .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                        .build();
                
                m_odometry = new SwerveDriveOdometry(
                        m_kinematics,
                        getGyroscopeRotation(),
                        new SwerveModulePosition[]{ m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition() }
                );

                swerveModulePositions = new SwerveModulePosition[]{m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()};
                everySecondTimer = new Timer();
                everySecondTimer.start();
        }

        public void setDriveSlow(double amt){
                MAX_VOLTAGE = 4/amt;
                drivetrainState = false;
        }

        public void setDriveNormal(){
                MAX_VOLTAGE = 11;
                drivetrainState = true;
        }

        public void xStance(){
                m_frontLeftModule.set(0, 44.8);
                m_frontRightModule.set(0, -44.8);
                m_backLeftModule.set(0, -44.8);
                m_backRightModule.set(0, 44.8);
        }

        public void unpitchRobot(){
                drive(new ChassisSpeeds(
                unpitchPIDController.calculate(getPitch(), 0),
                0,
                0
                ));
        }

        

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_gyro.zeroYaw();
        }

        public void calibrateGyro(){
                m_gyro.calibrate();
        }

        public double getPitch(){
                return m_gyro.getPitch();
        }

        public double getDriveVelocity(){
                return Math.abs(m_frontLeftModule.getDriveVelocity()) + Math.abs(m_frontRightModule.getDriveVelocity()) + Math.abs(m_backLeftModule.getDriveVelocity()) + Math.abs(m_backRightModule.getDriveVelocity())/4;
        }

        //returns what the drivetrain sees as gyro angle but as Rotation2d
        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(-m_gyro.getYaw());//360 - m_gyro.getYaw());
        }

        //returns what the drivetrain sees as gyro angle but as a double
        public double getGyroscopeRotationAsDouble(){
                return -m_gyro.getYaw();
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

        public SwerveModulePosition[] getSwerveModulePositions(){
                return swerveModulePositions;
        }

        public void updateSwerveModulePositions(){
                swerveModulePositions[0] = m_frontLeftModule.getPosition();
                swerveModulePositions[1] = m_frontRightModule.getPosition();
                swerveModulePositions[2] = m_backLeftModule.getPosition();
                swerveModulePositions[3] = m_backRightModule.getPosition();
        }

        public void resetPose2d(Pose2d pose){
                m_odometry.resetPosition(
                        getGyroscopeRotation(), 
                        new SwerveModulePosition[]{
                                m_frontLeftModule.getPosition(), 
                                m_frontRightModule.getPosition(), 
                                m_backLeftModule.getPosition(), 
                                m_backRightModule.getPosition()
                        },
                        pose
                );
        }

        public Pose2d getPose2d(){
                return m_odometry.getPoseMeters();
        }

        public SwerveDriveKinematics getKinematics(){
                return m_kinematics;
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
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
                states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                //updateSwerveModulePositions();
                m_odometry.update(
                        getGyroscopeRotation(),
                        new SwerveModulePosition[]{
                                m_frontLeftModule.getPosition(), 
                                m_frontRightModule.getPosition(), 
                                m_backLeftModule.getPosition(), 
                                m_backRightModule.getPosition()
                        }
                );

                // if(everySecondTimer.get()>=1){
                //         lastPitch = getPitch();
                //         everySecondTimer.reset();
                // }
        }
}
