package frc.robot.subsystems.turret.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.turret.Turret;

public class HardwareTurretFactory implements ITurretFactory{
    public final static int TURRET_MOTOR_PORT = 15;
    public Turret makeTurret(){
        var srx = new TalonSRXComponent(TURRET_MOTOR_PORT);
        srx.configForwardSoftLimitEnable(true);
        srx.configReverseSoftLimitEnable(true);
        srx.configForwardSoftLimitThreshold(8782);
        srx.configReverseSoftLimitThreshold(-8564);
        srx.configForwardSoftLimitThreshold(7500);
        srx.configReverseSoftLimitThreshold(-7500);
        return new Turret(srx, 1);
    }
}
