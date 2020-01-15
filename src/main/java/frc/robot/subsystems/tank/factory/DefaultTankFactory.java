/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.tank.factory;

import frc.robot.components.hardware.VictorSPComponent;
import frc.robot.subsystems.tank.Tank;

/**
 * Add your docs here.
 */
public class DefaultTankFactory {
    public Tank createTank(){
        return new Tank(new VictorSPComponent(0),new VictorSPComponent(2),new VictorSPComponent(1), new VictorSPComponent(3));
    }
}
