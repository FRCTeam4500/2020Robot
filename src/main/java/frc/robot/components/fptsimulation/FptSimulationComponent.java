/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.fptsimulation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class FptSimulationComponent {
    protected NetworkTable baseTable;
    public final String BASE_TABLE_KEY = "FPTSimulation";

    public FptSimulationComponent() {
        baseTable = NetworkTableInstance.getDefault().getTable(BASE_TABLE_KEY);
    }
}
