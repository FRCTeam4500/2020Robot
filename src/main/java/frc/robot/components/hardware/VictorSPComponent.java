package frc.robot.components.hardware;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.components.ISpeedSetterComponent;

public class VictorSPComponent extends VictorSP implements ISpeedSetterComponent {

    public VictorSPComponent(int deviceNumber) {
        super(deviceNumber);
    }

}