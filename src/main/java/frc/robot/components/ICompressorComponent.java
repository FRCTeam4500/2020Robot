/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

/**
 * An interface for all compressors. This usually refers to
 * {@link edu.wpi.first.wpilibj.Compressor Compressor}, but can also refer to
 * simulated compressors such as
 * {@link frc.robot.components.virtual.VirtualCompressorComponent
 * VirtualCompressorComponent}.
 */
public interface ICompressorComponent {
    /**
     * Start the compressor, if not already running.
     */
    void start();

    /**
     * Stop the compressor, if currently running.
     */
    void stop();

    /**
     * Returns whether the compressor is running or not.
     * 
     * @return is the compressor running
     */
    boolean enabled();

    /**
     * Toggles the compressor state from running to not running, and vice versa.
     */
    default void toggle() {
        if (enabled()) {
            stop();
        } else {
            start();
        }
    }
}
