package frc.robot.subsystems.turret;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISmartMotorComponent;

public class Turret extends SubsystemBase {
    private ISmartMotorComponent turnMotor;
    private double motorRotsPerTurretRots;
    public Turret(ISmartMotorComponent turnMotor, double motorRotsPerTurretRots) {
        this.turnMotor = turnMotor;
        this.motorRotsPerTurretRots = motorRotsPerTurretRots;
    }

    public void setTurretAngle(double turnAngle){
        this.turnMotor.setAngle(turnAngle);
    }
    public void setTurretOutput(double turretSpeed){
        turnMotor.setOutput(turretSpeed);
    }
    public double getTurretAngle(){
        return turnMotor.getAngle() / motorRotsPerTurretRots;
    }

}

