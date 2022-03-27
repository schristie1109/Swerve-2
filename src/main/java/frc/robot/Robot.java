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
        _drive = new Drive(this);
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() { timer.reset(); }

    @Override
    public void autonomousPeriodic() {
        /* An example of an autonomous path is included within this program. */
        double time = timer.get();

        if (time < 3) {
            _drive.run(0, -0.5, 0);
        }
        else {
            _drive.stop();
        }
    }

    @Override
    public void teleopInit() {
        _drive.stop();
    }

    @Override
    public void teleopPeriodic() {
        _swerve.teleopPeriodic();
        _drive.teleopPeriodic();
    }

    @Override  
    public void disabledInit() {
        // Stop Drivetrain
        _drive.stop();
    }

    @Override
    public void testPeriodic() {
        // Display NavPod information to the console
        _navpod.setAutoUpdate(0.2, update -> System.err.printf("h: %f, x: %f, sx: %f, y: %f, ys: %f\n", update.h, update.x, update.sx, update.y, update.sy));
    }

    // Get Gyroscope heading
    public double getGyroscope() { return heading; }
}
