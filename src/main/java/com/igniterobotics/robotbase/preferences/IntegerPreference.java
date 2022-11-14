package com.igniterobotics.robotbase.preferences;


public class IntegerPreference extends RobotPreference<Integer> {
    private int defaultValue;

    public IntegerPreference(String key) {
        this(key, 0);
    }

    public IntegerPreference(String key, int defaultValue) {
        super(key);
        this.defaultValue = defaultValue;

        preferences.initInt(key,defaultValue);
    }

    public int getValue() {
        return preferences.getInt(super.key, defaultValue);
    }

    @Override
    public Integer get() {
        return getValue();
    }
}
