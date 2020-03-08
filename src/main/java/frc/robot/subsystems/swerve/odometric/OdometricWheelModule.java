/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.components.ISmartMotorComponent;
import frc.robot.subsystems.swerve.kinematic.KinematicWheelModule;
import static frc.robot.utility.ExtendedMath.getShortestRadianToTarget;

/**
 * Add your docs here.
 */
public class OdometricWheelModule extends KinematicWheelModule {

    protected ISmartMotorComponent driveMotor;
    protected ISmartMotorComponent angleMotor;
    protected boolean wheelWrapEnabled = true;
    protected boolean wheelInversionEnabled = true;

    public OdometricWheelModule(ISmartMotorComponent angleMotor, ISmartMotorComponent driveMotor, Translation2d translationFromSwerveCenter,
            double maxSurfaceSpeed, double wheelDiameter,
            double angleRotsPerMotorRots, double driveRotsPerMotorRots) {
            super(angleMotor, driveMotor, translationFromSwerveCenter, maxSurfaceSpeed, wheelDiameter, angleRotsPerMotorRots, driveRotsPerMotorRots);
            this.driveMotor = driveMotor;
            this.angleMotor = angleMotor;
    }
    

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getAngularVelocity() * driveRotsPerMotorRots / 2
                / Math.PI * Math.PI * wheelDiameter,
                new Rotation2d(angleMotor.getAngle() * angleRotsPerMotorRots));
    }

    @Override
    public void drive(SwerveModuleState state) {
        if (wheelWrapEnabled) {
            double currentAngle = angleMotor.getAngle() *angleRotsPerMotorRots;

            // Get the closest angle equivalent to the target angle. Otherwise the wheel
            // module is going
            // to spin a lot since it will be at
            // something like 3 pi and the target will be 0 pi; the motor will move 3 pi
            // instead of just
            // 1 pi without this bit
            double shortestRadianToTarget = getShortestRadianToTarget(currentAngle, state.angle.getRadians());
            double targetAngle = shortestRadianToTarget + currentAngle;

            if (wheelInversionEnabled) {
                // Sometimes it will be easier to reverse the speed instead of rotating the
                // whole module by
                // pi radians.
                // These lines determine whether this is needed.
                double oppositeAngle = targetAngle + Math.PI;
                double shortestDistanceToOppositeAngle = getShortestRadianToTarget(currentAngle, oppositeAngle);
                double finalAngle;
                double finalSpeed;
                if (Math.abs(shortestDistanceToOppositeAngle) < Math.abs(shortestRadianToTarget)) {
                    finalAngle = currentAngle + shortestDistanceToOppositeAngle;
                    finalSpeed = -state.speedMetersPerSecond;
                } else {
                    finalAngle = currentAngle + shortestRadianToTarget;
                    finalSpeed = state.speedMetersPerSecond;
                }
                state.angle = new Rotation2d(finalAngle);
                state.speedMetersPerSecond = finalSpeed;
            }else{
                state.angle = new Rotation2d(targetAngle);
            }
        }
        super.drive(state);
    }
    public void coast(){
        driveMotor.setOutput(0);
    }
}
