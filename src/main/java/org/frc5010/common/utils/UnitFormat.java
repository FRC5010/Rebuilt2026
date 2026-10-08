package org.frc5010.common.utils;

public final class UnitFormat {

  private UnitFormat() {}

  /** Formats a length in meters as "F ft I N/D in", rounded to the nearest 1/8 inch. */
  public static String metersToFeetAndFractionalInches(double meters) {
    double totalInches = meters / 0.0254;
    int eighths = (int) Math.round(totalInches * 8);
    int feet = eighths / 96;
    int wholeInches = (eighths % 96) / 8;
    int fracNum = eighths % 8;

    if (fracNum == 0) {
      return String.format("%d ft %d in", feet, wholeInches);
    }
    int gcd = gcd(fracNum, 8);
    return String.format("%d ft %d %d/%d in", feet, wholeInches, fracNum / gcd, 8 / gcd);
  }

  private static int gcd(int a, int b) {
    return b == 0 ? a : gcd(b, a % b);
  }
}
