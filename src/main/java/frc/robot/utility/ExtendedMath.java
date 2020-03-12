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
        var norm = b.getNorm();
        if(norm == 0){
            return 0;
        }else{
        return dot(a,b)/norm;
        }
    }
    public static Translation2d normalize(Translation2d a){
        return a.div(a.getNorm());
    }
    public static double withHardDeadzone(double value, double deadzone){
        if(Math.abs(value) < deadzone){
            return 0;
        }else{
            return value;
        }
    }
    public static double withContinuousDeadzone(double input, double slope, double deadzone){
        if(input <= -deadzone){
            return (input + deadzone) * slope;
        }else if(-deadzone < input && input < deadzone){
            return 0;
        }else{
            return (input - deadzone) * slope;
        }
    }
        /**
     * A custom mod function which returns a remainder with the same sign as the dividend. This is
     * different from using {@code %}, which returns the remainder with the same sign as the
     * divisor.
     * 
     * @param a the dividend
     * @param n the divisor
     * @return the remainder with the same sign as {@code a}
     */
    public static double customMod(double a, double n) {
        return a - Math.floor(a / n) * n;
    }

    /**
     * Calculates the shortest radian to a given angle, assuming that all angles that are 2 pi away
     * from each other are equivalent.
     * 
     * @param currentAngle the starting angle
     * @param targetAngle  the final angle
     * @return the smallest difference and direction between these two angles
     */
    public static double getShortestRadianToTarget(double currentAngle, double targetAngle) {
        double actualDifference = targetAngle - currentAngle;
        double shortestDifference = customMod(actualDifference + Math.PI, 2 * Math.PI) - Math.PI;
        return shortestDifference;
    }
}
