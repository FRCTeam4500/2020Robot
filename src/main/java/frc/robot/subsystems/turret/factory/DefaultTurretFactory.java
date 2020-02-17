package frc.robot.subsystems.turret.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretMap;

public class DefaultTurretFactory implements ITurretFactory{
    public Turret makeTurret(){
        return new Turret(new TalonSRXComponent(TurretMap.TURRET_MOTOR_PORT));
    }
}
