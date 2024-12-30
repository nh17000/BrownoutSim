package frc.lib.util;

import java.util.Map;
import java.util.TreeMap;

public class LerpTable {
    private TreeMap<Double, Double> datapoints = new TreeMap<>();

    public void addPoint(double key, double value) {
        datapoints.put(key, value);
    }

    public void addPoints(double[] keys, double[] values) {
        for(int i = 0; i < keys.length; i++)
            datapoints.put(keys[i], values[i]);
    }

    public double interpolate(double input) {
        var y = datapoints.ceilingEntry(input);
        var x = datapoints.floorEntry(input);
    
        if (x == null && y == null) {
            return 0;
        } else if (x == null) {
            return y.getValue();
        } else if (y == null) {
            return x.getValue();
        } else if (x.getKey().equals(y.getKey())) {
            return x.getValue();
        } else {
            return x.getValue() + (y.getValue() - x.getValue()) * (input - x.getKey()) / (y.getKey() - x.getKey());
        }
    }

    public double inverseInterpolate(double targetY) {
        Map.Entry<Double, Double> lower = null;
        Map.Entry<Double, Double> upper = null;
    
        // Iterate through the datapoints to find bounds for the target y value
        for (Map.Entry<Double, Double> entry : datapoints.entrySet()) {
            if (entry.getValue() <= targetY) {
                lower = entry; // Largest y less than or equal to targetY
            }
            if (entry.getValue() >= targetY) {
                upper = entry; // Smallest y greater than or equal to targetY
                break;
            }
        }
    
        if (lower == null && upper == null) {
            return 0;
        } else if (lower == null) {
            return upper.getKey();
        } else if (upper == null) {
            return lower.getKey();
        } else if (lower.getValue().equals(upper.getValue())) {
            return lower.getKey();
        } else {
            return lower.getKey() + (targetY - lower.getValue()) * 
                   (upper.getKey() - lower.getKey()) / (upper.getValue() - lower.getValue());
        }
    }    
}