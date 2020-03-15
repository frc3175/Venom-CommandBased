package frc.robot.utilities;

import frc.robot.utilities.Utils;

public class Utils {
    public static double ensureRange(double v, double min, double max) {
        return Math.min(Math.max(v, min), max);
    }

    public static double negPowTwo(double v) {
        return (v != 0) ? Math.pow(v, 2) * (Math.abs(v) / v) : 0;
    }

    public static double keepRange(double v) {
        if (v > 1.0) {
            return 1.0;
        } else {
            return v;
        }
    }

    /**
     * 
     * @param val       Joystick axis
     * @param threshold Actual deadband value
     * @return joystick axis
     */
    public static double deadband(double val, double threshold) {
        if (Math.abs(val) < threshold) {
            return 0;
        }
        return val;
    }

    public static double addAngle(double ang1, double ang2) {
        return Math.acos(Math.cos(ang1) * Math.cos(ang2));
    }

    public static double climberEncoderCalculator() {
        return ((Math.PI / 2) * (7 * 2048));
    }

    public static double degToRad(double deg) {
        return deg * Math.PI / 180.0;
    }

    public static double inchesToEncs(double inches) {
        // Based on wheel circumfrence, perform calculation for inches.
        return inches;
    }

    public static double expoDeadzone(double input, double deadzone, double exponent) {
        if (Math.abs(input) <= deadzone)
            return 0;
        double deadzoned = (input - Math.signum(input) * deadzone) / (1 - deadzone);
        // System.out.println(deadzoned);
        double expoed = Math.pow(Math.abs(deadzoned), exponent) * Math.signum(deadzoned);

        return expoed;
    }
}