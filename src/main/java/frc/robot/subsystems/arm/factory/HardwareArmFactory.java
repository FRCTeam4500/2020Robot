package frc.robot.subsystems.arm.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.arm.Arm;

public class HardwareArmFactory implements IArmFactory {
    /**
     *
     */
    private static final int ARM_MOTOR_PORT = 8;

    public Arm makeArm(){

        var srx = new TalonSRXComponent(ARM_MOTOR_PORT);
        srx.configReverseSoftLimitEnable(true);
        srx.configReverseSoftLimitThreshold(-3800);
        srx.configForwardSoftLimitEnable(true);
        srx.configForwardSoftLimitThreshold(0);
        return new Arm(srx);
    }
}
