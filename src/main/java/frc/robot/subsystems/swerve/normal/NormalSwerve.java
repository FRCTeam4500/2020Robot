/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.normal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IGyroComponent;

/**
 * The subsystem representing the swerve drive on the robot. A swerve drive is a
 * drivetrain which allows independent movement along the X, Y, and rotational
 * axes. This implementation consists of four {@link NormalWheelModule}s on each
 * corner of the robot.
 */
public class NormalSwerve extends SubsystemBase {

    private double length, width;
    private NormalWheelModule fl, fr, bl, br;
    private IGyroComponent gyro;

    /**
     * The "rotation coefficient" represents a value that converts rotational input
     * into wheel module speed. Normally, the rotational input is measured in
     * radians/second of the robot's rotation. However, translating this angular
     * velocity into linear velocity is impossible without knowing the size of the
     * wheels and the speed at which they rotate. Therefore, we turn these values
     * into a coefficient which can be tuned as needed.
     */
    private double rotationCoefficient = 1;

    /**
     * This is the diagnoal length of the swerve drive from the center to any
     * corner.
     */
    private double r;

    /**
     * Creates a swerve drive. All the following parameters are relative to a
     * top-down perspective, with the robot facing upwards.
     * 
     * @param length the horizontal length between wheel modules
     * @param width  the vertical length between wheel modules
     * @param fl     the front left wheel module
     * @param fr     the front right wheel module
     * @param bl     the back left wheel module
     * @param br     the back right wheel module
     * @param gyro   the gyro which measures the rotation of the robot
     */
    public NormalSwerve(double length, double width, NormalWheelModule fl, NormalWheelModule fr, NormalWheelModule bl, NormalWheelModule br,
            IGyroComponent gyro) {
        this.length = length;
        this.width = width;
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        r = Math.sqrt(width * width + length * length) / 2;
        this.gyro = gyro;
    }

    /**
     * Moves the robot relative to itself, meaning that no angle transformations are
     * done. When it is given the command to move left, the robot moves towards its
     * left, and the same applies to all other directions. The resulting speed is
     * clamped to fit the limits of motors.
     * 
     * @param x the horizontal speed of the robot
     * @param y the vertical speed of the robot
     * @param w the rotational speed of the robot (negative is clockwise, positive
     *          is counter clockwise)
     */
    public void moveRobotCentric(double x, double y, double w) {
        double wkr = w * rotationCoefficient / r;

        // We sometimes get weird issues with math functions recieving negative zeros,
        // so this code
        // is here to make all zeros positive.
        if (y == 0) {
            y = +0;
        }

        double a = x + wkr * width / 2;
        double b = y + wkr * length / 2;
        double c = y - wkr * length / 2;
        double d = x - wkr * width / 2;

        double flSpeed = Math.sqrt(c * c + a * a);
        double frSpeed = Math.sqrt(a * a + b * b);
        double blSpeed = Math.sqrt(c * c + d * d);
        double brSpeed = Math.sqrt(b * b + d * d);

        // Lower all motor speeds in case any are above 1.0, so that the robot still
        // moves in the
        // right direction (although not as quickly.)
        double maxSpeed = Math.max(1, Math.max(flSpeed, Math.max(frSpeed, Math.max(blSpeed, brSpeed))));
        flSpeed /= maxSpeed;
        frSpeed /= maxSpeed;
        blSpeed /= maxSpeed;
        brSpeed /= maxSpeed;

        double blAngle = Math.atan2(-a, c);
        double brAngle = Math.atan2(-a, b);
        double flAngle = Math.atan2(-d, c);
        double frAngle = Math.atan2(-d, b);

        fl.drive(flAngle, flSpeed);
        fr.drive(frAngle, frSpeed);
        bl.drive(blAngle, blSpeed);
        br.drive(brAngle, brSpeed);
    }

    /**
     * Moves the robot relative to the given angle. This angle acts as the
     * rotational offset of the robot so that if the robot were commanded to move
     * horizontally, it would move perpendicular to this angle despite whatever
     * rotation the robot is at.
     * 
     * @param angle the angle which the robot moves relative to
     * @see {@link #moveRobotCentric(double, double, double)}
     */
    public void moveAngleCentric(double x, double y, double w, double angle) {
        double s = Math.sin(angle);
        double c = Math.cos(angle);

        double xnew = x * c - y * s;
        double ynew = x * s + y * c;

        moveRobotCentric(xnew, ynew, w);
    }

    /**
     * Moves the robot relative to the field. If the robot were commanded to move
     * horizontally, it would move across the field despite its rotation. The
     * robot's gyro is utilized to determine the offset from the field.
     * 
     * @see #moveAngleCentric(double, double, double, double)
     * @see #moveRobotCentric(double, double, double)
     */
    public void moveFieldCentric(double x, double y, double w) {
        moveAngleCentric(x, y, w, gyro.getAngle());
    }

    /**
     * Resets the gyro of the robot, effectively making its current angle zero. This
     * is equivalent to calling {@link IGyroComponent#reset()}.
     */
    public void resetGyro() {
        gyro.reset();
    }
}
