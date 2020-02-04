/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.hardware;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.components.ISpeedSetterComponent;

/**
 * Add your docs here.
 */
public class VictorSPComponent extends VictorSP implements ISpeedSetterComponent{

    public VictorSPComponent(int channel) {
        super(channel);
    }
}
