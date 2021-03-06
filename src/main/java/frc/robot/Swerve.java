// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.util.Constants.*;

public class Swerve {
    private NavPod _navpod;

    private double heading;

    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public Swerve() {
        _navpod = new NavPod();

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk3SwerveModuleHelper.createNeo(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk3SwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk3SwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk3SwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
        );

        // Check if the NavPod is connected to RoboRIO
        if (_navpod.isValid())
        {
            NavPodConfig config = new NavPodConfig();
            config.cableMountAngle = 0;
            config.fieldOrientedEnabled = false;
            config.initialHeadingAngle = 90;
            config.mountOffsetX = 0;
            config.mountOffsetY = 0;
            config.rotationScaleFactorX = 0.05;
            config.rotationScaleFactorY = 0.0;
            config.translationScaleFactor = 0.00748;
            _navpod.setConfig(config);
            
            // Report values to the console
            config = _navpod.getConfig();
            System.err.printf("config.cableMountAngle: %f\n", config.cableMountAngle);
            System.err.printf("config.fieldOrientedEnabled: %b\n", config.fieldOrientedEnabled);
            System.err.printf("config.initialHeadingAngle: %f\n", config.initialHeadingAngle);
            System.err.printf("config.mountOffsetX: %f in\n", config.mountOffsetX);
            System.err.printf("config.mountOffsetY: %f in\n", config.mountOffsetY);
            System.err.printf("config.rotationScaleFactorX: %f\n", config.rotationScaleFactorX);
            System.err.printf("config.rotationScaleFactorY: %f\n", config.rotationScaleFactorY);
            System.err.printf("config.translationScaleFactor: %f\n", config.translationScaleFactor);

            _navpod.resetH(0);
            _navpod.resetXY(0, 0);

            // Update console with NavPod info every 10ms
            _navpod.setAutoUpdate(0.02, update -> heading = (double) update.h);
        }

        tab.addNumber("Gyroscope Angle", () -> heading);
        tab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        tab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
        tab.addBoolean("NavPod Status", () -> _navpod.isValid());
        tab.addNumber("NavPod Heading", () -> heading);
    }

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(heading));

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public void zeroGyroscope() {
        odometry.resetPosition(
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                Rotation2d.fromDegrees(heading)
        );
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public double getPoseX() {
        return odometry.getPoseMeters().getX();
    }

    public double getPoseY() {
        return odometry.getPoseMeters().getY();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }
    
    public void teleopPeriodic() {
        odometry.update(Rotation2d.fromDegrees(heading),
            new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
            new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
            new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
            new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
        );
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    public void lock() {
        frontLeftModule.set(0, 315 * (Math.PI / 180));
        frontRightModule.set(0, 45 * (Math.PI / 180));
        backLeftModule.set(0, 45 * (Math.PI / 180));
        backRightModule.set(0, 315 * (Math.PI / 180));
    }
}