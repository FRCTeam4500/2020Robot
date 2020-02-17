package frc.robot.subsystems.arm.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmMap;

public class DefaultArmFactory implements IArmFactory {
    public Arm makeArm(){
        return new Arm(new TalonSRXComponent(ArmMap.ARM_MOTOR_PORT));
    }
}
