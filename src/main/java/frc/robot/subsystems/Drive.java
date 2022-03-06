// Official Robot Code for The F.A.S.T Team 7056
// Developed by @schristie1109
// 2/27/2022

package frc.robot.subsystems;

import frc.robot.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;

public class Drive {
    private final Swerve drivetrain = new Swerve();
    XboxController _driver = new XboxController(0);

    double translationXSupplier = _driver.getRightX();
    double translationYSupplier = _driver.getRightY();
    double rotationSupplier = _driver.getLeftX();

    public void teleopPeriodic(boolean isFieldOriented) {
        double xT = 1.0;

        // Check for driver Right Trigger held
        if (_driver.getRawAxis(3) > 0.2) {
            xT = 0.65;
        }

        // Apply dummy mode to Joystick values
        double translationXPercent = translationXSupplier * xT;
        double translationYPercent = translationYSupplier * xT;
        double rotationPercent = rotationSupplier * xT;

        if (isFieldOriented) {

            // Run Drive command (Field Oriented Drive)
            drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            -modifyAxis(translationXPercent) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                            -modifyAxis(translationYPercent) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                            -modifyAxis(rotationPercent) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                            drivetrain.getRotation()));
        } else {

            // Run Drive command (Robot Oriented Drive)
            drivetrain.drive(
                    new ChassisSpeeds(
                            -modifyAxis(translationXPercent) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                            -modifyAxis(translationYPercent) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                            -modifyAxis(rotationPercent) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
        }
    }

    // Stop the drivetrain
    public void stop() {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
    // Run the drivetrain
    public void run(double[] driveArray) {
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -modifyAxis(driveArray[0]) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                        -modifyAxis(driveArray[1]) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                        -modifyAxis(driveArray[2]) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        drivetrain.getRotation()));
    }

    public void run(double x, double y, double z) {
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -modifyAxis(x) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                        -modifyAxis(y) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                        -modifyAxis(z) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        drivetrain.getRotation()));
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
        return deadband(Math.copySign(value * value, value), 0.1);
    }
}