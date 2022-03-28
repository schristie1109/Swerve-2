package frc.robot.utils;

public final class Constants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4953;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4953;
    
    // Front Left Module
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 20;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    // Front Right Module
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 21;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    // Back Left Module
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 22;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
    
    // Back Right Module
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 23;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 13;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
}