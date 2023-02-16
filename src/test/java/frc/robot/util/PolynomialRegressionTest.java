package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import lib.PolynomialRegression;

public class PolynomialRegressionTest {

  /**
   * Unit tests the {@code PolynomialRegression} data type.
   */
  @Test
  public void testRegression() {
    double[] x = {10, 20, 40, 80, 160, 200};
    double[] y = {100, 350, 1500, 6700, 20160, 40000}; 
    PolynomialRegression regression = new PolynomialRegression(x, y, 3);

    assertEquals(regression.beta(3), 0.0092, 0.0001);
    assertEquals(regression.beta(2), -1.6395, 0.0001);
    assertEquals(regression.beta(1), 168.9232, 0.0001);
    assertEquals(regression.beta(0), -2113.7306, 0.0001);
  }

}
