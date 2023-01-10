package com.igniterobotics.robotbase.preferences;

import java.util.Collection;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;

public abstract class RobotPreference<T> implements Supplier<T> {
    protected static Preferences preferences = Preferences.getInstance();
    
    protected String key;

    public RobotPreference(String key) {
        this.key = key;
    }

    public void remove() {
        preferences.remove(key);
    }

    public static void removeAll() {
        preferences.removeAll();
    }

    public static Collection<String> getKeys() {
        return preferences.getKeys();
    }
}
