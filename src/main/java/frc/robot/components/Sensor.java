/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components;
import frc.robot.subsystems.indexer.Indexer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class Sensor {
    String sensorName = " ";
    public Sensor(String sensorName) {
        this.sensorName = sensorName;
    }
    public boolean registersBall(){
        return SmartDashboard.getNumber("sensor0", Double.POSITIVE_INFINITY) < 17;
    }
}
