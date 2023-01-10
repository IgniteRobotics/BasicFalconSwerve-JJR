package com.igniterobotics.robotbase.preferences;

public class StringPreference extends RobotPreference<String> {
    private String defaultValue;

    public StringPreference(String key) {
        this(key, "");
    }

    public StringPreference(String key, String defaultValue) {
        super(key);
        this.defaultValue = defaultValue;

        preferences.initString(key, defaultValue);
    }

    public String getValue() {
        return preferences.getString(super.key, defaultValue);
    }

    @Override
    public String get() {
        return getValue();
    }
}
