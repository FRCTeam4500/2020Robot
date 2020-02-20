package frc.robot.subsystems.turret.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretMap;

public class DefaultTurretFactory implements ITurretFactory{
    public Turret makeTurret(){

        TalonSRXComponent turnMotor = new TalonSRXComponent(TurretMap.TURRET_MOTOR_PORT);
        turnMotor.configForwardSoftLimitThreshold(9176, 0);
        turnMotor.configReverseSoftLimitThreshold(-7646, 0);
        turnMotor.configForwardSoftLimitEnable(true, 0);
        turnMotor.configReverseSoftLimitEnable(true, 0);
        return new Turret(turnMotor);
        
    }
}
