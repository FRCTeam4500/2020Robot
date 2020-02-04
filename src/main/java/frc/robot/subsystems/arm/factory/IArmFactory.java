package frc.robot.subsystems.arm.factory;

import frc.robot.components.IAngleSetterComponent;
import frc.robot.subsystems.arm.Arm;

public interface IArmFactory {
    public Arm makeArm();
}
