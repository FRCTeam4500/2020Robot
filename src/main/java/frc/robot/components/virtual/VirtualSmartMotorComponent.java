/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.ISmartMotorComponent;

/**
 * Add your docs here.
 */
public class VirtualSmartMotorComponent implements ISmartMotorComponent, Sendable {

    private String name = "Unnammed Virtual Smart Motor Component";
    private String subsystem = "Ungrouped";


    private double angle = 0.0; // radians
    private double velocity = 0.0; // radians / sec
    private double minAbsoluteVelocity = 1;
    private double maxAbsoluteVelocity = 6.0;
    private double acceleration = 0.0;
    private double minAbsoluteAcceleration = 0.0;
    private double maxAbsoluteAcceleration = 6.0;

    private PIDController angleController;
    private PIDController velocityController;
    private Notifier notifier;
    private double period = 0.02; // s

    public void setMinAbsoluteVelocity(double velocity) {
        minAbsoluteVelocity = velocity;
    };

    public void setMinAbsoluteAccelertion(double acceleration) {
        minAbsoluteAcceleration = acceleration;
    }

    public void setMaxAbsoluteVelocity(double velocity) {
        maxAbsoluteVelocity = velocity;
    }

    public void setMaxAbsoluteAcceleration(double acceleration) {
        maxAbsoluteAcceleration = acceleration;
    }

    public double getMinAbsoluteVelocity() {
        return minAbsoluteVelocity;
    }

    public double getMaxAbsoluteVelocity() {
        return maxAbsoluteVelocity;
    }

    public double getMinAbsoluteAcceleration() {
        return minAbsoluteAcceleration;
    }

    public double getMaxAbsoluteAcceleration() {
        return maxAbsoluteAcceleration;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setVelocityPIDF(double kP, double kI, double kD, double kF) {
        velocityController.setPID(kP, kI, kD, kF);
    }

    public void setPositionPIDF(double kP, double kI, double kD, double kF) {
        angleController.setPID(kP, kI, kD, kF);
    }

    public VirtualSmartMotorComponent() {
        angleController = new PIDController(1, 0, 0, new PIDSource() {

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public double pidGet() {
                return getAngle();
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }
        }, this::setAcceleration, period);
        velocityController = new PIDController(1, 0, 0, new PIDSource() {

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return getVelocity();
            }

        }, this::setAcceleration, period);

        angleController.setOutputRange(-Double.MAX_VALUE, Double.MAX_VALUE);
        velocityController.setOutputRange(-Double.MAX_VALUE, Double.MAX_VALUE);

        notifier = new Notifier(this::fixedUpdate);
        notifier.startPeriodic(period);
    }

    @Override
    public double getAngle() {
        return angle;
    }

    public void setTargetAngle(double angle) {
        angleController.setSetpoint(angle);
        angleController.enable();
        velocityController.reset();
    }

    public void setTargetVelocity(double velocity) {
        velocityController.setSetpoint(velocity);
        velocityController.enable();
        angleController.reset();
    }

    @Override
    public void setAngle(double angle) {
        setTargetAngle(angle);
    }

    /**
     * @param acceleration the acceleration to set
     */
    public void setAcceleration(double acceleration) {

        this.acceleration =
                specialClamp(minAbsoluteAcceleration, maxAbsoluteAcceleration, acceleration);
    }

    @Override
    public void setSpeed(double speed) {
        setTargetVelocity(velocity);
    }

    private void fixedUpdate() {
        velocity = specialClamp(minAbsoluteVelocity, maxAbsoluteVelocity,
                velocity += acceleration * period);
        angle += velocity * period;
    }

    public double getVelocity() {
        return velocity;
    }

    @Override
    public double getSpeed() {
        return getVelocity();
    }

    private double specialClamp(double min, double max, double value) {
        if (value > max) {
            return max;
        }
        if (-min < value && value < min) {
            return 0;
        }
        if (value < -max) {
            return -max;
        }
        return value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Acceleration", this::getAcceleration, this::setAcceleration);
        builder.addDoubleProperty("Max Absolute Acceleration", this::getMaxAbsoluteAcceleration,
                this::setMaxAbsoluteAcceleration);
        builder.addDoubleProperty("Min Absolute Acceleration", this::getMinAbsoluteAcceleration,
                this::setMinAbsoluteAccelertion);
        builder.addDoubleProperty("Max Absolute Velocity", this::getMaxAbsoluteVelocity,
                this::setMaxAbsoluteVelocity);
        builder.addDoubleProperty("Min Absolute Velocity", this::getMinAbsoluteVelocity,
                this::setMinAbsoluteVelocity);
        builder.addDoubleProperty("Target Velocity", velocityController::getSetpoint,
                this::setTargetVelocity);
        builder.addDoubleProperty("Target Angle", angleController::getSetpoint,
                this::setTargetAngle);
        builder.addBooleanProperty("Is In Velocity Mode", velocityController::isEnabled, null);
        builder.addBooleanProperty("Is In Position Mode", angleController::isEnabled, null);

        builder.addDoubleProperty("Velocity kP", velocityController::getP,
                velocityController::setP);
        builder.addDoubleProperty("Velocity kI", velocityController::getI,
                velocityController::setI);
        builder.addDoubleProperty("Velocity kD", velocityController::getD,
                velocityController::setD);
        builder.addDoubleProperty("Velocity kF", velocityController::getF,
                velocityController::setF);

        builder.addDoubleProperty("Position kP", angleController::getP, angleController::setP);
        builder.addDoubleProperty("Position kI", angleController::getI, angleController::setI);
        builder.addDoubleProperty("Position kD", angleController::getD, angleController::setD);
        builder.addDoubleProperty("Position kF", angleController::getF, angleController::setF);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void setName(String name) {
        this.name = name;
    }

    @Override
    public String getSubsystem() {
        return subsystem;
    }

    @Override
    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }
}
