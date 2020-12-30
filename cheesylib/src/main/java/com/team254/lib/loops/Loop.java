package com.team254.lib.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {

    void onStart(Phase phase);

    void onLoop(double timestamp);

    void onStop(double timestamp);

    public enum Phase {
        DISABLED,
        TELEOP,
        AUTONOMOUS,
        TEST,
    } 
}
