// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.*;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder.ControllerImplementation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

        public static final double MAX_VOLTAGE = 12.0;
        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        //  The formula for calculating the theoretical maximum velocity is:
        //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
        //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
        //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
        //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
        The maximum velocity of the robot in meters per second.
        <p>
        This is a measure of how fast the robot should be able to drive in a straight line.
        */
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

        // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        // FIXME Remove if you are using a Pigeon
        // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
        // FIXME Uncomment if you are using a NavX
        //  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;


        private AHRS gyro;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                // There are 4 methods you can call to create your swerve modules.
                // The method you use depends on what motors you are using.
                //
                // import frc.robot.subsystems.GyroscopeAccelerometer; frc.robot.subsystems.GyroscopeAccelerometer;per.createFalcon500(...)
                //   Your module has two Falcon 500s on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createNeo(...)
                //   Your module has two NEOs on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createFalcon500Neo(...)
                //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
                //
                // Mk3SwerveModuleHelper.createNeoFalcon500(...)
                //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
                //
                // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

                // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
                // you MUST change it. If you do not, your code will crash on startup.
                // FIXME Setup motor configuration
                gyro = new AHRS(SPI.Port.kMXP);

                Mk4ModuleConfiguration moduleConfig = new Mk4ModuleConfiguration();
                moduleConfig.setDriveCurrentLimit(40);
                moduleConfig.setSteerCurrentLimit(20);
                moduleConfig.setNominalVoltage(12);

                m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                        moduleConfig, //tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0)
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        FRONT_LEFT_MODULE_DRIVE_MOTOR, 
                        FRONT_LEFT_MODULE_STEER_MOTOR, 
                        FRONT_LEFT_MODULE_STEER_ENCODER, 
                        FRONT_LEFT_MODULE_STEER_OFFSET
                );

                m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                        moduleConfig, //tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0)
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
                        FRONT_RIGHT_MODULE_STEER_MOTOR, 
                        FRONT_RIGHT_MODULE_STEER_ENCODER, 
                        FRONT_RIGHT_MODULE_STEER_OFFSET
                );
                
                m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                        moduleConfig, //tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0)
                        com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio.L1, 
                        BACK_LEFT_MODULE_DRIVE_MOTOR, 
                        BACK_LEFT_MODULE_STEER_MOTOR, 
                        BACK_LEFT_MODULE_STEER_ENCODER, 
                        BACK_LEFT_MODULE_STEER_OFFSET
                );

                m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                        moduleConfig, //tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0)
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
                return Rotation2d.fromDegrees(gyro.getYaw());
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Gyroscope Position from Swerve", 0);
                SmartDashboard.putNumber("Gyroscope Position from Swerve", gyro.getYaw());
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                //SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                SwerveDriveKinematics.desaturateWheelSpeeds(states,MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        }
}
