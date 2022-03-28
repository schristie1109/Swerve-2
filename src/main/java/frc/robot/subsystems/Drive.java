// Official Robot Code for The F.A.S.T Team 7056
// Developed by @schristie1109
// 2/27/2022

package frc.robot.subsystems;

import frc.robot.Swerve;
import frc.robot.utils.Utilities;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;

public class Drive {
    XboxController _driver = new XboxController(0);

    double translationXSupplier = _driver.getRightX();
    double translationYSupplier = _driver.getRightY();
    double rotationSupplier = _driver.getLeftX();

    private Swerve drivetrain;
    public Drive() {

        drivetrain = new Swerve();
    }

    public void teleopPeriodic() {
        double xT = 1.0;

        // Check for driver Right Trigger held
        if (_driver.getRawAxis(3) > 0.2) {
            xT = 0.65;
        }

        // Apply dummy mode to Joystick values
        double translationXPercent = translationXSupplier * xT;
        double translationYPercent = translationYSupplier * xT;
        double rotationPercent = rotationSupplier * xT;

        if (_driver.getLeftBumper()) {
            drivetrain.lock();
        }

        // Run Drive command (Robot Oriented Drive)
        drivetrain.drive(
            new ChassisSpeeds(
                    -modifyAxis(translationXPercent) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                    -modifyAxis(translationYPercent) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                    -modifyAxis(rotationPercent) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    }

    // Stop the drivetrain
    public void stop() {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public void run(double x, double y, double z) {
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -modifyAxis(x) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                        -modifyAxis(y) * Swerve.MAX_VELOCITY_METERS_PER_SECOND,
                        -modifyAxis(z) * Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        drivetrain.getRotation()));
    }

    private static double modifyAxis(double value) {
        return Utilities.deadband(Math.copySign(value * value, value));
    }
}