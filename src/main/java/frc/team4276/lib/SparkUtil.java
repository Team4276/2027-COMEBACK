// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.lib;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.util.Signal;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  // REVLib telemetry getters (encoder position/velocity, applied output, bus voltage,
  // output current, ...) now return Signal<Double> instead of a raw double, carrying
  // per-value validity via isValid() rather than the old device-wide spark.getLastError()
  // side channel. Both are checked here to preserve the original device-health intent.

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, Supplier<Signal<Double>> supplier, DoubleConsumer consumer) {
    Signal<Double> signal = supplier.get();
    if (signal.isValid() && spark.getLastError() == REVLibError.kOk) {
      consumer.accept(signal.get());
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, List<Supplier<Signal<Double>>> suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.size()];
    for (int i = 0; i < suppliers.size(); i++) {
      Signal<Double> signal = suppliers.get(i).get();
      if (!signal.isValid() || spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
      values[i] = signal.get();
    }
    consumer.accept(values);
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }
}
