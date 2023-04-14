package frc.robot.util;

import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTreeMap extends TreeMap<Double, Double> {
  public double get(double key) {
    Entry<Double, Double> upperEntry = super.ceilingEntry(key);
    Entry<Double, Double> lowerEntry = super.floorEntry(key);
    if (upperEntry == null && lowerEntry == null) return Double.NaN;
    else if (upperEntry == null) return lowerEntry.getValue();
    else if (lowerEntry == null) return upperEntry.getValue();
    else
      return lerp(
          key,
          lowerEntry.getKey(),
          lowerEntry.getValue(),
          upperEntry.getKey(),
          upperEntry.getValue());
  }

  public double lerp(double x, double x0, double y0, double x1, double y1) {
    if (x0 == x1) return y0;
    return (x - x0) / (x1 - x0) * (y1 - y0) + y0;
  }
}
