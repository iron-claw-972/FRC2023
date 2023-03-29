package frc.robot.util;

import java.util.ArrayList;

public class StatisticsUtil {
  public static double[] arrayListToArray(ArrayList<Double> arrayList){
    double[] array = new double[arrayList.size()];
    for(int i = 0; i < arrayList.size(); i++){
        array[i] = arrayList.get(i);
    }
    return array;
  }

  public static double mean(ArrayList<Double> data){
    return mean(arrayListToArray(data));
  }

  public static double mean(double[] data){
    double mean = 0;
    for (int i = 0; i < data.length; i++) {
      mean += data[i];
    }
    mean /= data.length;
    return mean;
  }

  public static double stdDev(ArrayList<Double> data){
    return stdDev(arrayListToArray(data));
  }

  public static double stdDev(double[] data){
    if(data.length == 0){
      return 0;
    }
    
    double mean = mean(data);
    
    double total = 0;
    for (int i = 0; i < data.length; i++) {
      total += Math.pow(data[i] - mean, 2);
    }
    return Math.sqrt(total / (data.length-1));
  }
}
