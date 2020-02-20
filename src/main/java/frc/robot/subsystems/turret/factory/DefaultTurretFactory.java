package frc.robot.subsystems.turret.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretMap;

public class DefaultTurretFactory implements ITurretFactory{
    public Turret makeTurret(){

        TalonSRXComponent turnMotor = new TalonSRXComponent(TurretMap.TURRET_MOTOR_PORT);
        turnMotor.configForwardSoftLimitThreshold(9176);
        turnMotor.configReverseSoftLimitThreshold(-7646);
        turnMotor.configForwardSoftLimitEnable(true);
        turnMotor.configReverseSoftLimitEnable(true);
        return new Turret(turnMotor);
        
    }
}
