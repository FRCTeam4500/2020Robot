/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.fptsimulation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.components.IGyroComponent;

/**
 * Add your docs here.
 */
public class GyroSimulationComponent extends FptSimulationComponent implements IGyroComponent{

    NetworkTable gyrosTable;
    NetworkTable gyro;
    public final String ANGLE_GETTER_KEY = "AngleGetter";
    NetworkTableEntry angleTableEntry;
    public GyroSimulationComponent(int id){
        super();
        gyrosTable = baseTable.getSubTable("Gyros");
        gyro = gyrosTable.getSubTable(Integer.toString(id));
        angleTableEntry = gyro.getEntry(ANGLE_GETTER_KEY);
    }
    private double offset = 0.0;
    @Override
    public double getAngle() {
        return angleTableEntry.getDouble(0.0) - offset;
    }

    @Override
    public void reset() {
        offset = getAngle();
    }
}
