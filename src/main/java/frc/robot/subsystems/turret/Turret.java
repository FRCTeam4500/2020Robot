package frc.robot.subsystems.turret;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleGetterSetterComponent;

public class Turret extends SubsystemBase {
    private IAngleGetterSetterComponent turnMotor;
    public Turret(IAngleGetterSetterComponent turnMotor) {
        this.turnMotor = turnMotor;
    }

    public void setTurretAngle(double turnAngle){
        turnMotor.setAngle(turnAngle);
    }

    public double getTurretAngle(){
        return turnMotor.getAngle();
    }

}

