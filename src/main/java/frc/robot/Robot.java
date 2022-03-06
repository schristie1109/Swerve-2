// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Official Robot Code for The F.A.S.T Team, Team 7056
// Developed by @schristie1109

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    double heading;
    Timer timer = new Timer();
    ShuffleboardTab tab = Shuffleboard.getTab("Robot Data");

    // Initialize Subsystem Classes
    NavPod _navpod;
    Drive _drive;
    Swerve _swerve;

    /* This function is run when the robot is first started up and should be used for any
    initialization code. */
    @Override
    public void robotInit() {
        _navpod = new NavPod();
        _drive = new Drive();
        _swerve = new Swerve();

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
            _navpod.setAutoUpdate(0.10, update -> heading = (double) update.h);

            // Debug command to setup NavPod
            // _navpod.setAutoUpdate(0.10, update -> System.err.printf("h: %f, x: %f, sx: %f, y: %f, ys: %f\n", update.h, update.x, update.sx, update.y, update.sy));
        }
    }

    @Override
    public void robotPeriodic() {
        // Display Shuffleboard Info
        tab.addBoolean("NavPod Status", () -> _navpod.isValid());
        tab.addNumber("NavPod Heading", () -> heading);
    }
    
    @Override
    public void autonomousInit() { timer.reset(); }

    @Override
    public void autonomousPeriodic() {
        /* Two examples of an autonomous path are included in this program */
        double time = timer.get();

        // {+X, +Y, ->R}
        double[] driveArray = {0, 0, 0};
        _drive.run(driveArray);

        if (time < 8) {
            driveArray[1] = -0.5;
        }

        /*

        if (time < 8) {
            _drive.run(0, -0.5, 0);
        }
        else {
            _drive.stop();
        }

        */
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        // Run Swerve Code
        _swerve.teleopPeriodic();

        // Run Drivetrain
        _drive.teleopPeriodic(true);
    }

    @Override
    public void disabledInit() {
        // Stop Drivetrain
        _drive.stop();
    }

    // Get Gyroscope heading
    public double getGyroscope() { return heading; }
}
