package frc.robot.subsystems.swerve.kinematic;

import edu.wpi.first.wpilibj.util.Units;

public class KinematicSwerveMap {
    public static final double ANGLE_P = 1.03858;
    public static final double ANGLE_I = 0.004;
    public static final double ANGLE_D = 8;
    public static final double ANGLE_F = 0.51;
    public static final int ANGLE_V = 4012;
    public static final int ANGLE_A = 4012;
    public static final int BL_ANGLE_PORT = 8;
    public static final int BR_ANGLE_PORT = 2;
    public static final int FL_ANGLE_PORT = 6;
    public static final int FR_ANGLE_PORT = 3;
    public static final int BL_SPEED_PORT = 9;
    public static final int BR_SPEED_PORT = 1;
    public static final int FL_SPEED_PORT = 7;
    public static final int FR_SPEED_PORT = 4;
    public static final double SWERVE_LENGTH = 2;
    public static final double SWERVE_WIDTH = 2;
<<<<<<< Updated upstream
    public static final double MAX_SURFACE_SPEED = 4.8;
=======

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0);
    public static final double DRIVE_LEFTWARD = Units.inchesToMeters(24.5 - 1.75);
    public static final double DRIVE_FORWARD = Units.inchesToMeters(24.5 - 1.75);
>>>>>>> Stashed changes
}
