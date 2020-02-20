package frc.robot.subsystems.turret.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretMap;

public class DefaultTurretFactory implements ITurretFactory{

    public Turret makeTurret(){
        TalonSRXComponent srx = new TalonSRXComponent(TurretMap.TURRET_MOTOR_PORT);
        srx.configForwardSoftLimitThreshold(TurretMap.TURRET_MAXMIMUM_TICKS);
        srx.configReverseSoftLimitThreshold(TurretMap.TURRET_MINIMUM_TICKS);
        srx.configForwardSoftLimitEnable(true);
        srx.configReverseSoftLimitEnable(true);
        return new Turret(srx);
    }
}
