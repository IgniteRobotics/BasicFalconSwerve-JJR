package com.igniterobotics.robotbase.calc;

public class Measurement {
    public final Unit unit;
    public final double value;

    public Measurement(Unit unit, double value) {
        this.unit = unit;
        this.value = value;
    }

    public double getAsMeters() {
        switch(unit) {
            case INCHES:
                return Units.inToM(value);
            case CENTIMETERS:
                return Units.cmToM(value);
            default:
                return value;
        }
    }

    public double getAsCentimeters() {
        switch(unit) {
            case INCHES:
                return Units.inToCm(value);
            case METERS:
                return Units.mToCm(value);
            default:
                return value;
        }
    }

    public double getAsInches() {
        switch(unit) {
            case METERS:
                return Units.mToIn(value);
            case CENTIMETERS:
                return Units.cmToIn(value);
            default:
                return value;
        }
    }

    public double getAsRadians() {
        switch(unit) {
            case DEGREES:
                return Units.degToRad(value);
            default:
                return value;
        }
    }

    public double getAsDegrees() {
        switch(unit) {
            case RADIANS:
                return Units.radToDeg(value);
            default:
                return value;
        }
    }

    public static enum Unit {
        METERS,
        INCHES,
        CENTIMETERS,
        RADIANS,
        DEGREES
    }
}
