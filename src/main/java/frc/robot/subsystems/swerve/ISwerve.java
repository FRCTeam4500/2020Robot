package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISwerve extends Subsystem {
    void moveFieldCentric(double x, double y, double w);
    void moveRobotCentric(double x, double y, double w);
    void moveAngleCentric(double x, double y, double z, double angle);
    void resetGyro();
}
