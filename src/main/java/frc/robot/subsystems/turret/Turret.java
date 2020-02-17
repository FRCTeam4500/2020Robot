package frc.robot.subsystems.turret;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleSetterComponent;
import frc.robot.components.ISmartMotorComponent;

public class Turret extends SubsystemBase {
    private IAngleSetterComponent turnMotor;
    public Turret(IAngleSetterComponent turnMotor) {
        this.turnMotor = turnMotor;
    }

    public void setTurretAngle(double turnAngle){
        this.turnMotor.setAngle(turnAngle);
    }

}

