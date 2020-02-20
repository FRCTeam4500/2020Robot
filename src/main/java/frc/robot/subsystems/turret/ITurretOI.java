package frc.robot.subsystems.turret;

public interface ITurretOI {
    public double getTurretDesiredAngle();
    public double getTurretAngle();
    public void setTurretDesiredAngle(double angle);
}
