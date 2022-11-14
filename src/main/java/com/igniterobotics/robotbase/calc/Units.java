package com.igniterobotics.robotbase.calc;

public class Units {
    public static double inToM(double in) {
        return in * 0.0254;
    }

    public static double mToIn(double m) {
        return m / 0.0254;
    }

    public static double inToCm(double in) {
        return in * 2.54;
    }

    public static double cmToIn(double cm) {
        return cm / 2.54;
    }

    public static double cmToM(double cm) {
        return cm / 100;
    }

    public static double mToCm(double m) {
        return m * 100;
    }

    public static double degToRad(double deg) {
        return deg * (Math.PI / 180);
    }

    public static double radToDeg(double rad) {
        return rad * (180 / Math.PI);
    }
}
