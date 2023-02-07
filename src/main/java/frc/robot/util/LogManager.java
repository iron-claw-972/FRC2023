package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LogManager {

  private static DataLog log = DataLogManager.getLog();

  static ArrayList<DoubleLogEntry> doubleLogs = new ArrayList<DoubleLogEntry>();
  static ArrayList<DoubleSupplier> doubleValues = new ArrayList<DoubleSupplier>();

  static ArrayList<DoubleArrayLogEntry> doubleArrayLogs = new ArrayList<DoubleArrayLogEntry>();
  static ArrayList<DoubleSupplier[]> doubleArrayValues = new ArrayList<DoubleSupplier[]>();

  static ArrayList<BooleanLogEntry> boolLogs = new ArrayList<BooleanLogEntry>();
  static ArrayList<BooleanSupplier> boolValues = new ArrayList<BooleanSupplier>();
  static ArrayList<IntegerLogEntry> intLogs = new ArrayList<IntegerLogEntry>();
  static ArrayList<IntSupplier> intValues = new ArrayList<IntSupplier>();

  public static void addDouble(String name, DoubleSupplier logged) {
    DoubleLogEntry myDoubleLog = new DoubleLogEntry(log, name);
    doubleLogs.add(myDoubleLog);
    doubleValues.add(logged);
  }

  public static void addDoubleArray(String name, DoubleSupplier[] logged) {
    DoubleArrayLogEntry myDoubleLog = new DoubleArrayLogEntry(log, name);
    doubleArrayLogs.add(myDoubleLog);
    doubleArrayValues.add(logged);
  }

  /**
   * When called, this function creates an integer data entry and stores it in an
   * arraylist.
   */

  public static void addInt(String name, IntSupplier logged) {

    IntegerLogEntry IntegerLog = new IntegerLogEntry(log, name);
    intLogs.add(IntegerLog);
    intValues.add(logged);
  }

  /**
   * When called, this function creates a double data entry and stores it in an
   * arraylist.
   */
  public static void addBool(String name, BooleanSupplier logged) {

    BooleanLogEntry BooleanLog = new BooleanLogEntry(log, name);
    boolLogs.add(BooleanLog);
    boolValues.add(logged);
  }

  /**
   * For every entry in the data entry array list, it logs a value of the
   * corresponding index in the value array list.
   */

  public static void log() {

    for (int i = 0; i < doubleLogs.size(); i++) {
      doubleLogs.get(i).append(doubleValues.get(i).getAsDouble());
    }
    for (int i = 0; i < doubleArrayLogs.size(); i++) {
      double[] values = new double[doubleArrayValues.get(i).length];
      for (int j = 0; j < doubleArrayValues.get(i).length; j++) {
        values[j] = doubleArrayValues.get(i)[j].getAsDouble();
      }
      doubleArrayLogs.get(i).append(values);
    }
    for (int i = 0; i < intLogs.size(); i++) {
      intLogs.get(i).append(intValues.get(i).getAsInt());
    }
    for (int i = 0; i < boolLogs.size(); i++) {
      boolLogs.get(i).append(boolValues.get(i).getAsBoolean());
    }
  }
}