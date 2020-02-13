/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * This is a simple container for math functions which are useful, but are not included in the
 * standard {@link Math} class.
 */

// Feel free to add any useful math-related functions to this class. Please do not implement any
// non-static methods.
public class ExtendedMath {
    /**
     * Clamps an output between {@code min} and {@code max}. "Clamping" refers to restricting a
     * value between a minimum and a maximum. If the given value is below the minimum, the returned
     * value is equal to the minimum. If the given value is above the maximum, the returned value is
     * equal to the maximum. If neither of these conditions are met, the given value is returned as
     * is.
     * 
     * @param min    the value minimum
     * @param max    the value maximum
     * @param output the value to be clamped
     * @return the clamped value
     */
    public static double clamp(double min, double max, double output) {
        return Math.min(max, Math.max(min, output));
    }
    public static double dot(Translation2d a, Translation2d b){
        return a.getX()*b.getX()+a.getY()*b.getY();
    }
    public static double angleBetween(Translation2d a, Translation2d b){
        return Math.acos(dot(a,b)/(a.getNorm()*b.getNorm()));
    }
    public static double scalarProjectionOf(Translation2d a, Translation2d b){
        return dot(a,b)/b.getNorm();
    }
    public static Translation2d normalize(Translation2d a){
        return a.div(a.getNorm());
    }
}
