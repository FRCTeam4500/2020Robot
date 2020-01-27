package frc.robot.subsystems.turret.factory;

import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretMap;

public class DefaultTurretFactory implements ITurretFactory{
    public Turret makeTurret(){
        return new Turret(new TalonFXComponent(TurretMap.TURRET_MOTOR_PORT));
    }
}
