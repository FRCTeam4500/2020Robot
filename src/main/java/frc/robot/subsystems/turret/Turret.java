package frc.robot.subsystems.turret;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISmartMotorComponent;

public class Turret extends SubsystemBase {
    private ISmartMotorComponent turnMotor;
    public Turret(ISmartMotorComponent turnMotor) {
        this.turnMotor = turnMotor;
    }

    public void setTurretAngle(double turnAngle){
        this.turnMotor.setAngle(turnAngle);
    }
    public void setTurretOutput(double turretSpeed){
        turnMotor.setOutput(turretSpeed);
    }

}

