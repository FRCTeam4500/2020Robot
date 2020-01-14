/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

/**
 * A data container for a 3D transform.
 */
public class Transform3D {
    private double x;
    private double y;
    private double z;
    private double pitch;
    private double yaw;
    private double roll;

    /**
     * @return the x
     */
    public double getX() {
        return x;
    }

    /**
     * @return the y
     */
    public double getY() {
        return y;
    }

    /**
     * @return the z
     */
    public double getZ() {
        return z;
    }

    /**
     * @return the pitch
     */
    public double getPitch() {
        return pitch;
    }

    /**
     * @return the yaw
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * @return the roll
     */
    public double getRoll() {
        return roll;
    }

    public Transform3D(double x, double y, double z, double pitch, double yaw, double roll) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
    }

}
