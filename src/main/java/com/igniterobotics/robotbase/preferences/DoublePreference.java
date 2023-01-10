package com.igniterobotics.robotbase.preferences;


public class DoublePreference extends RobotPreference<Double> {
    private double defaultValue;

    public DoublePreference(String key) {
        this(key, 0.0);
    }

    public DoublePreference(String key, double defaultValue) {
        super(key);
        this.defaultValue = defaultValue;

        preferences.initDouble(key, defaultValue);
    }

    public double getValue() {
        return preferences.getDouble(super.key, defaultValue);
    }

    @Override
    public Double get() {
        return getValue();
    }
}
