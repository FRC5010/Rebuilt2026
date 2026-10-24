package frc.robot.rebuilt.audit;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.util.Units;
import frc.robot.rebuilt.FieldConstants;
import java.io.File;
import java.io.IOException;

/**
 * Trench geometry in the blue-origin frame, derived from {@link FieldConstants}.
 *
 * <p>Two criteria are used for "the robot is under the trench structure":
 *
 * <ul>
 *   <li><b>centre</b>: the robot centre is inside the structure's X extent (the hood sits near the
 *       robot centre)
 *   <li><b>body</b>: any part of the bumper overlaps the structure in X (more conservative)
 * </ul>
 *
 * In Y the robot must be inside the trench corridor: y &lt; opening width (bottom/right trench) or
 * y &gt; field width - opening width (top/left trench).
 */
public final class TrenchGeometry {
  private TrenchGeometry() {}

  public static final double HUB_X = FieldConstants.LinesVertical.hubCenter;
  public static final double OPP_HUB_X = FieldConstants.LinesVertical.oppHubCenter;
  /**
   * Physical depth of the trench structure (47 in). Deliberately not {@code
   * FieldConstants.LeftTrench.depth}: that value is also used to tune how early the trench guard
   * reacts and was reduced to 24 in, so the audit would otherwise stop describing the real field.
   */
  public static final double DEPTH = Units.inchesToMeters(47.0);
  public static final double OPENING_WIDTH = FieldConstants.LeftTrench.openingWidth;
  public static final double FIELD_WIDTH = FieldConstants.fieldWidth;

  /** Half the bumper length, from the drivetrain JSON (square 33 in bumpers on this robot). */
  public static final double BUMPER_HALF = readBumperHalf();

  /** margin for {@link #zone}: robot centre only */
  public static final double CENTER = 0.0;
  /** margin for {@link #zone}: whole bumper */
  public static final double BODY = BUMPER_HALF;

  private static double readBumperHalf() {
    try {
      JsonNode c =
          new ObjectMapper()
              .readTree(new File("src/main/deploy/rebuilt_robot/akit_swerve_drivetrain.json"))
              .get("constants");
      double len = inches(c.get("bumperFrameLength"));
      double wid = inches(c.get("bumperFrameWidth"));
      return Math.max(len, wid) / 2.0;
    } catch (IOException e) {
      throw new IllegalStateException("cannot read bumper size", e);
    }
  }

  private static double inches(JsonNode unitValue) {
    String uom = unitValue.get("uom").asText();
    if (!uom.startsWith("inch")) throw new IllegalStateException("unexpected unit " + uom);
    return Units.inchesToMeters(unitValue.get("val").asDouble());
  }

  /**
   * @param margin extra X half-extent beyond the structure (use {@link #CENTER} or {@link #BODY})
   * @return e.g. "bottom-near", "top-far", or null if the point is not under a trench
   */
  public static String zone(double x, double y, double margin) {
    String[] names = {"near", "far"};
    double[] centres = {HUB_X, OPP_HUB_X};
    for (int i = 0; i < 2; i++) {
      if (Math.abs(x - centres[i]) < DEPTH / 2 + margin) {
        if (y < OPENING_WIDTH) return "bottom-" + names[i];
        if (y > FIELD_WIDTH - OPENING_WIDTH) return "top-" + names[i];
      }
    }
    return null;
  }
}
