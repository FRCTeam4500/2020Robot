/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.SendableBase;

/**
 * The base decorator for all dashboard decorator components. By extending this class, a decorator
 * can display the properties of the decorated objects onto any interface that accepts a
 * {@link edu.wpi.first.wpilibj.Sendable Sendable}, such Shuffleboard. A decorator is intended to
 * wrap around a component and passed around instead of the component itself.
 */
public abstract class DashboardDecorator extends SendableBase {

    /**
     * Creates a decorator with the respective parameters. In Shuffleboard, the decorator is defined
     * as [subsystem]/[name] and decorator properties are defined as [subsystem]/[name]/[property],
     * allowing for easy organization.
     * 
     * @param name      the name of the decorated component
     * @param subsystem the subsystem which the component is part of
     */
    public DashboardDecorator(String name, String subsystem) {
        setName(name);
        setSubsystem(subsystem);
    }
}
