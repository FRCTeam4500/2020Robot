/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.ICompressorComponent;

/**
 * A {@link DashboardDecorator} for any {@link ICompressorComponent} component.
 */
public class CompressorDashboardDecorator extends DashboardDecorator implements ICompressorComponent {

    private boolean lastIsCompressing = false;
    private ICompressorComponent compressor;

    /**
     * See {@link DashboardDecorator#DashboardDecorator(String, String)} for more
     * details.
     * 
     * @param compressor the {@link ICompressorComponent} to decorate.
     */
    public CompressorDashboardDecorator(String name, String subsystem, ICompressorComponent compressor) {
        super(name + " " + "Compressor Component", subsystem);
        this.compressor = compressor;
        send();
    }

    @Override
    public void start() {
        lastIsCompressing = true;
        compressor.start();
    }

    @Override
    public void stop() {
        lastIsCompressing = false;
        compressor.stop();
    }

    /**
     * Gets the last command sent to the compressor. This value is independent from
     * whether the compressor is actually running or not, so it is used to verify
     * that the compressor is being commanded at all. If the compressor is broken,
     * and {@link #start()} has been invoked, this method will return true, but
     * {@link #enabled()} will return false.
     * 
     * @return the last sent compressor command
     */
    public boolean getLastIsCompressing() {
        return lastIsCompressing;
    }

    @Override
    public boolean enabled() {
        return compressor.enabled();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Last Is Compressing", this::getLastIsCompressing, null);
        builder.addBooleanProperty("Real Is Compressing", this::enabled, null);
    }
}
