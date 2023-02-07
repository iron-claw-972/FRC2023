package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LogManager {
    
    private static DataLog log = DataLogManager.getLog();
   
    static ArrayList<DoubleLogEntry> doubleLogs = new ArrayList<DoubleLogEntry>(); 
    static ArrayList<DoubleSupplier> doubleValues = new ArrayList<DoubleSupplier>();
    static ArrayList<BooleanLogEntry> boolLogs = new ArrayList<BooleanLogEntry>(); 
    static ArrayList<BooleanSupplier> boolValues = new ArrayList<BooleanSupplier>();
    static ArrayList<IntegerLogEntry> intLogs = new ArrayList<IntegerLogEntry>(); 
    static ArrayList<IntSupplier> intValues = new ArrayList<IntSupplier>();

    /**
     * Logs a double from a function
     * 
     * @param name The name of the log. Use / to create subdirectories, and keep names unique.
     * @param logged A double supplier of the value to be logged. Can be created by using a lambda on a function that returns a double.
     */
    public static void addDouble(String name, DoubleSupplier logged) { 
        DoubleLogEntry myDoubleLog = new DoubleLogEntry(log, name);
        doubleLogs.add(myDoubleLog);
        doubleValues.add(logged);
    }

    /**
     * Logs a int from a function
     * 
     * @param name The name of the log. Use / to create subdirectories, and keep names unique.
     * @param logged An int supplier of the value to be logged. Can be created by using a lambda on a function that returns a int.
     */
    public static void addInt(String name, IntSupplier logged) { 
     
        IntegerLogEntry IntegerLog = new IntegerLogEntry(log, name);
        intLogs.add(IntegerLog);
        intValues.add(logged);
    }

    /** Logs a boolean from a function
    * 
    * @param name The name of the log. Use / to create subdirectories, and keep names unique.
    * @param logged An boolean supplier of the value to be logged. Can be created by using a lambda on a function that returns a boolean.
    */
    public static void addBool(String name, BooleanSupplier logged) { 
        BooleanLogEntry BooleanLog = new BooleanLogEntry(log, name);
        boolLogs.add(BooleanLog);
        boolValues.add(logged);
    }

    /**
     * Logs all the values that have been collected. Should be called periodically. 
     */
    public static void log() {
        for (int i = 0; i < doubleLogs.size(); i++)
        {
            doubleLogs.get(i).append(doubleValues.get(i).getAsDouble());
        }
        for (int i = 0; i < intLogs.size(); i++)
        {
            intLogs.get(i).append(intValues.get(i).getAsInt());
        }
        for (int i = 0; i < boolLogs.size(); i++)
        {
            boolLogs.get(i).append(boolValues.get(i).getAsBoolean());
        }
    }
}
