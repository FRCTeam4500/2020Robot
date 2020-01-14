/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

import frc.robot.utility.Transform3D;

/**
 * An interface for any component which calculates the position and rotation of
 * the robot.
 */
public interface I3DSupplierComponent {

    Transform3D getTransform();
}
