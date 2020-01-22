/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.fptsimulation;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.components.ISmartMotorComponent;

/**
 * Add your docs here.
 */
public class SmarterSmartMotorSimulationComponent extends FptSimulationComponent implements ISmartMotorComponent{

    public final String MOTOR_TABLES_KEY = "SmarterSmartMotors",
    ANGLE_GETTER_KEY = "AngleGetter",
    ANGLE_SETTER_KEY = "AngleSetter",
    OUTPUT_SETTER_KEY = "OutputSetter",
    OUTPUT_GETTER_KEY = "OutputGetter",
    ANGULAR_VELOCITY_SETTER_KEY = "AngularVelocitySetter",
    ANGULAR_VELOCITY_GETTER_KEY = "AngularVelocityGetter";

    private NetworkTableEntry 
    angleGetterEntry,
    angleSetterEntry,
    outputSetterEntry,
    outputGetterEntry,
    angularVelocitySetterEntry,
    angularVelocityGetterEntry;

    public SmarterSmartMotorSimulationComponent(int id){
        super();
        var motorTable = baseTable.getSubTable(MOTOR_TABLES_KEY).getSubTable(Integer.toString(id));
        angleGetterEntry = motorTable.getEntry(ANGLE_GETTER_KEY);
        angleSetterEntry = motorTable.getEntry(ANGLE_SETTER_KEY);
        outputSetterEntry = motorTable.getEntry(OUTPUT_SETTER_KEY);
        outputGetterEntry = motorTable.getEntry(OUTPUT_GETTER_KEY);
        angularVelocitySetterEntry = motorTable.getEntry(ANGULAR_VELOCITY_SETTER_KEY);
        angularVelocityGetterEntry = motorTable.getEntry(ANGULAR_VELOCITY_GETTER_KEY);
    }

    @Override
    public double getAngle() {
        return angleGetterEntry.getDouble(Double.NaN);
    }

    @Override
    public void setAngle(double angle) {
        angleSetterEntry.forceSetDouble(Double.NaN);
    }

    @Override
    public void setOutput(double output) {
        outputSetterEntry.forceSetNumber(output);
    }

    @Override
    public double getOutput() {
        return outputGetterEntry.getDouble(Double.NaN);
    }

    @Override
    public double getAngularVelocity() {
        return angularVelocityGetterEntry.getDouble(Double.NaN);
    }

    @Override
    public void setAngularVelocity(double velocity) {
        angularVelocitySetterEntry.forceSetNumber(velocity);
    }
}
