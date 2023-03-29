package frc.robot.util;

import java.util.ArrayList;

public class StatisticsUtil {
  public static double stdDev(ArrayList<Double> data){
    double[] dataArray = new double[data.size()];
    for(int i = 0; i < data.size(); i++){
        dataArray[i] = data.get(i);
    }
    return stdDev(dataArray);
  }

  public static double stdDev(double[] data){
    if(data.length == 0){
      return 0;
    }
          
    double mean = 0;
    for (int i = 0; i < data.length; i++) {
      mean += data[i];
    }
    mean /= data.length;
    double total = 0;
    for (int i = 0; i < data.length; i++) {
      total += Math.pow(data[i] - mean, 2);
    }
    return Math.sqrt(total / (data.length-1));
  }
}
