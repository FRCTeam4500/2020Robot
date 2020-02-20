/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.normal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleGetterComponent;
import frc.robot.components.IAngleSetterComponent;
import frc.robot.components.IOutputSetterComponent;
import static frc.robot.utility.ExtendedMath.getShortestRadianToTarget;
/**
 * A subsystem which represents the combination of a speed motor and an angle motor. This class is
 * used by {@link NormalSwerve} drives in order to move the robot in any direction. Currently, there is no
 * real reason for this class to be a subsystem other than the fact that it was a subsystem in
 * previous code. This class implements angle wrapping and speed inversion optimizations.
 */
public class NormalWheelModule extends SubsystemBase {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    protected IAngleSetterComponent angleSetter;
    protected IOutputSetterComponent speedSetter;
    protected IAngleGetterComponent angleGetter;
    /**
     * This variable exists so that the wheel module can always make wheel wrapping and speed
     * inversion optimizations, even if no encoder for the angle exists. The wheel module assumes
     * that its current angle is the last angle it was set to.
     */
    private double lastAngle;

    /**
     * Create a wheel module with the given angle and speed controllers
     * 
     * @param angleSetter the component which controls the angle of the wheel module
     * @param speedSetter the component which controls the speed of the wheel module
     */
    public NormalWheelModule(IAngleSetterComponent angleSetter, IOutputSetterComponent speedSetter) {
        this.angleSetter = angleSetter;
        this.speedSetter = speedSetter;
    }

    /**
     * Similar to {@link NormalWheelModule#WheelModule(IAngleSetterComponent, IOutputSetterComponent)}, but
     * with an added component to measure the current angle of the wheel module. This allows for
     * better wheel wrapping and speed inversion optimizations.
     * 
     * @param angleGetter the component which measures the angle of the wheel module
     */
    public NormalWheelModule(IAngleSetterComponent angleSetter, IOutputSetterComponent speedSetter,
            IAngleGetterComponent angleGetter) {
        this(angleSetter, speedSetter);
        this.angleGetter = angleGetter;
    }

    /**
     * Sets the angle and speed of this wheel module. This method implements wheel wrapping, meaning
     * that it will automatically consider equivalent angles to be on target. For example, if the
     * wheel module was currently at 2 pi, and the desired angle was 4 pi, the wheel module would
     * not make a full rotation to match the target angle. This method also implements speed
     * inverting, so that if rotating the wheel module to the opposite angle and reversing the speed
     * is faster, then the wheel module does exactly that. For example, if the wheel module is at
     * 1/4 pi, and the desired angle was 1 pi, the wheel module would move to 0 pi and reverse its
     * speed.
     * 
     * @param angle the desired angle of the wheel module
     * @param speed the desired speed of the wheel module
     */
    public void drive(double angle, double speed) {
        if (angleGetter != null) {
            lastAngle = angleGetter.getAngle();
        }
        // Get the closest angle equivalent to the target angle. Otherwise the wheel
        // module is going
        // to spin a lot since it will be at
        // something like 3 pi and the target will be 0 pi; the motor will move 3 pi
        // instead of just
        // 1 pi without this bit
        double shortestRadianToTarget = getShortestRadianToTarget(lastAngle, angle);
        double targetAngle = shortestRadianToTarget + lastAngle;

        // Sometimes it will be easier to reverse the speed instead of rotating the
        // whole module by
        // pi radians.
        // These lines determine whether this is needed.
        double oppositeAngle = targetAngle + Math.PI;
        double shortestDistanceToOppositeAngle =
                getShortestRadianToTarget(lastAngle, oppositeAngle);
        double finalAngle;
        double finalSpeed;
        if (Math.abs(shortestDistanceToOppositeAngle) < Math.abs(shortestRadianToTarget)) {
            finalAngle = lastAngle + shortestDistanceToOppositeAngle;
            finalSpeed = -speed;
        } else {
            finalAngle = lastAngle + shortestRadianToTarget;
            finalSpeed = speed;
        }

        angleSetter.setAngle(finalAngle);
        speedSetter.setOutput(finalSpeed);
        lastAngle = finalAngle;

    }

}
