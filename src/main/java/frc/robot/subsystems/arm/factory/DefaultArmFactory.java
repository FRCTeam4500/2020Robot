package frc.robot.subsystems.arm.factory;

import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmMap;

public class DefaultArmFactory implements IArmFactory {
    public Arm makeArm(){
        return new Arm(new TalonFXComponent(ArmMap.ARM_MOTOR_PORT));
    }
}
