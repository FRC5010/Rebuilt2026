package frc.robot.rebuilt.audit;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

/**
 * Builds an {@link AutoTimeline} from a B-Line path JSON.
 *
 * <p>B-Line semantics used: translation targets are {@code waypoint} and {@code translation}
 * elements, driven in order. An {@code event_trigger} fires when the robot's progress along the
 * segment between the nearest translation targets before and after it reaches {@code t_ratio} (and
 * immediately if the robot is already past that segment). Constraint ordinals index the translation
 * targets, and the speed limit of ordinal k applies to travel toward target k.
 *
 * <p>Timing model (fastest plausible, so lead times are conservative): the robot drives straight
 * segments at max(previous target limit, this target limit); the 0.45 m handoff radius, curvature
 * and accelerations are ignored.
 */
public final class BLineAudit {
  private static final ObjectMapper MAPPER = new ObjectMapper();

  private final Path pathsDir;
  private final double defaultMaxVelocity;

  /**
   * @param autosDir the deploy/autos directory (contains config.json and paths/)
   */
  public BLineAudit(Path autosDir) throws IOException {
    this.pathsDir = autosDir.resolve("paths");
    JsonNode cfg = MAPPER.readTree(autosDir.resolve("config.json").toFile());
    this.defaultMaxVelocity =
        cfg.get("kinematic_constraints").get("default_max_velocity_meters_per_sec").asDouble();
  }

  public List<String> pathNames() throws IOException {
    try (Stream<Path> s = Files.list(pathsDir)) {
      return s.map(p -> p.getFileName().toString())
          .filter(n -> n.endsWith(".json"))
          .map(n -> n.substring(0, n.length() - ".json".length()))
          .sorted()
          .toList();
    }
  }

  public AutoTimeline timeline(String pathName) throws IOException {
    File f = pathsDir.resolve(pathName + ".json").toFile();
    JsonNode root = MAPPER.readTree(f);
    JsonNode els = root.get("path_elements");

    // translation targets
    List<double[]> targets = new ArrayList<>();
    for (JsonNode e : els) {
      String type = e.get("type").asText();
      if (type.equals("waypoint")) {
        JsonNode t = e.get("translation_target");
        targets.add(new double[] {t.get("x_meters").asDouble(), t.get("y_meters").asDouble()});
      } else if (type.equals("translation")) {
        targets.add(new double[] {e.get("x_meters").asDouble(), e.get("y_meters").asDouble()});
      }
    }
    int n = targets.size();
    double[] vmax = new double[n];
    Arrays.fill(vmax, defaultMaxVelocity);
    JsonNode cons =
        root.has("constraints") ? root.get("constraints").get("max_velocity_meters_per_sec") : null;
    if (cons != null) {
      for (JsonNode c : cons) {
        for (int k = c.get("start_ordinal").asInt();
            k <= c.get("end_ordinal").asInt() && k < n;
            k++) {
          vmax[k] = c.get("value").asDouble();
        }
      }
    }

    // segment k goes from target k-1 to target k; cumulative start time of each target
    double[] tAt = new double[n];
    double[] segTime = new double[n];
    for (int k = 1; k < n; k++) {
      double len =
          Math.hypot(
              targets.get(k)[0] - targets.get(k - 1)[0], targets.get(k)[1] - targets.get(k - 1)[1]);
      double v = Math.max(vmax[k - 1], vmax[k]);
      segTime[k] = len / v;
      tAt[k] = tAt[k - 1] + segTime[k];
    }
    double end = n > 0 ? tAt[n - 1] : 0.0;

    // events
    List<HoodCommand> cmds = new ArrayList<>();
    int seen = 0; // translation targets seen so far in element order
    for (JsonNode e : els) {
      String type = e.get("type").asText();
      if (type.equals("waypoint") || type.equals("translation")) {
        seen++;
      } else if (type.equals("event_trigger")) {
        int a = seen - 1; // previous target
        int b = seen; // next target
        double t;
        if (a < 0) t = 0.0;
        else if (b >= n) t = tAt[a];
        else t = tAt[a] + e.get("t_ratio").asDouble() * segTime[b];
        HoodCommand c =
            HoodCommand.ofName(
                t, e.get("lib_key").asText(), "event " + e.get("t_ratio").asDouble());
        if (c != null) cmds.add(c);
      }
    }

    AutoTimeline.Position pos =
        n == 0
            ? null
            : t -> {
              if (t <= 0) return targets.get(0).clone();
              for (int k = 1; k < n; k++) {
                if (t <= tAt[k]) {
                  double frac = segTime[k] == 0 ? 1.0 : (t - tAt[k - 1]) / segTime[k];
                  double[] p0 = targets.get(k - 1);
                  double[] p1 = targets.get(k);
                  return new double[] {
                    p0[0] + frac * (p1[0] - p0[0]), p0[1] + frac * (p1[1] - p0[1])
                  };
                }
              }
              return targets.get(n - 1).clone();
            };
    return new AutoTimeline(pathName, end, cmds, pos);
  }
}
