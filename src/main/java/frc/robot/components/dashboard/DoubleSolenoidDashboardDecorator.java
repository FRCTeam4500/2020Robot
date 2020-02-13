/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IDoubleSolenoidComponent;

/**
 * A {@link DashboardDecorator} for any {@link IDoubleSolenoidComponent}
 * component.
 */
public class DoubleSolenoidDashboardDecorator extends DashboardDecorator implements IDoubleSolenoidComponent {

    private boolean lastIsExtended = false;
    private IDoubleSolenoidComponent ds;

    /**
     * See {@link DashboardDecorator#DashboardDecorator(String, String)} for more
     * details.
     * 
     * @param ds the {@link IDoubleSolenoidComponent} component to decorate
     */
    public DoubleSolenoidDashboardDecorator(String name, String subsystem, IDoubleSolenoidComponent ds) {
        super(name, subsystem);
        this.ds = ds;
        send();
    }

    /**
     * Gets the last sent command to the solenoid. If the solenoid is broken, and
     * {@link #extend()} has been invoked, this method will return true, but
     * {@link #isExtended()} will return false.
     * 
     * @return the last sent command to the compressor
     */
    public boolean getLastIsExtended() {
        return lastIsExtended;
    }

    @Override
    public boolean isExtended() {
        return ds.isExtended();
    }

    @Override
    public void extend() {
        lastIsExtended = true;
        ds.extend();
    }

    @Override
    public void retract() {
        lastIsExtended = false;
        ds.retract();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Last Is Extended", this::getLastIsExtended, null);
        builder.addBooleanProperty("Real Is Extended", this::isExtended, null);
    }
}
