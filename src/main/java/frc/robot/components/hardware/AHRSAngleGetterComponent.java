/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.components.IGyroComponent;

/**
 * An {@link IGyroComponent} wrapper for {@link AHRS}.
 */
public class AHRSAngleGetterComponent extends AHRS implements IGyroComponent {

    /**
     * @see AHRS#AHRS(Port)
     */
    public AHRSAngleGetterComponent(Port serial_port_id) {
        super(serial_port_id);
    }

    @Override
    public double getAngle() {
        return Math.toRadians(super.getAngle());
    }
}
