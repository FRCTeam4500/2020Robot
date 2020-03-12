/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import edu.wpi.first.wpilibj.Notifier;

/**
 * Add your docs here.
 */
public class OutputRamper {
    private double output, period, maxAbsoluteOutputChangePerLoop, setpoint;
    private Notifier notifier;

    public OutputRamper(double initialOutput, double period, double maxAbsoluteOutputChangePerLoop, double setpoint) {
        this.output = initialOutput;
        this.period = period;
        this.maxAbsoluteOutputChangePerLoop = maxAbsoluteOutputChangePerLoop;
        this.setpoint = setpoint;
        notifier = new Notifier(this::execute);
    }
    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }
    private void execute(){
        output += ExtendedMath.clamp(-maxAbsoluteOutputChangePerLoop, maxAbsoluteOutputChangePerLoop, (setpoint - output));
    }
    public double getOutput(){
        return output;
    }

    public void start(){
        notifier.startPeriodic(period);
    }
    public void stop() {
        notifier.stop();
    }
    

}
