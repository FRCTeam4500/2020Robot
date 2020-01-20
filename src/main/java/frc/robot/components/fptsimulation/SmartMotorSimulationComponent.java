/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.fptsimulation;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.components.IAngleGetterComponent;
import frc.robot.components.IAngleSetterComponent;
import frc.robot.components.IOutputSetterComponent;

/**
 * Add your docs here.
 */
public class SmartMotorSimulationComponent extends FptSimulationComponent
        implements IAngleGetterComponent, IAngleSetterComponent, IOutputSetterComponent {
    public final String MOTOR_TABLES_KEY = "SmartMotors";
    public final String ANGLE_GETTER_COMPONENT_KEY = "AngleGetterComponent";
    public final String ANGLE_SETTER_COMPONENT_KEY = "AngleSetterComponent";
    public final String SPEED_SETTER_COMPONENT_KEY = "SpeedSetterComponent";
    public final String MOTOR_MODE_KEY = "MotorMode";
    private NetworkTableEntry angleGetterEntry;
    private NetworkTableEntry angleSetterEntry;
    private NetworkTableEntry speedSetterEntry;
    private NetworkTableEntry motorModeEntry;

    public SmartMotorSimulationComponent(int motorId) {
        super();
        var motorTable =
                baseTable.getSubTable(MOTOR_TABLES_KEY).getSubTable(Integer.toString(motorId));
        angleGetterEntry = motorTable.getEntry(ANGLE_GETTER_COMPONENT_KEY);
        angleSetterEntry = motorTable.getEntry(ANGLE_SETTER_COMPONENT_KEY);
        speedSetterEntry = motorTable.getEntry(SPEED_SETTER_COMPONENT_KEY);
        motorModeEntry = motorTable.getEntry(MOTOR_MODE_KEY);
    }

    @Override
    public void setOutput(double speed) {
        speedSetterEntry.setNumber(speed);
        motorModeEntry.setString("Speed");
    }

    @Override
    public void setAngle(double angle) {
        angleSetterEntry.setNumber(angle);
        motorModeEntry.setString("Angle");
    }

    @Override
    public double getAngle() {
        return angleGetterEntry.getDouble(0.0);
    }
}
