package frc.robot.rebuilt.audit;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

/**
 * Builds an {@link AutoTimeline} from a PathPlanner {@code .auto} file.
 *
 * <p>Timing model (deliberately the FASTEST plausible case, so lead times are conservative): each
 * path takes its minimum-time rest-to-rest trapezoid using the path's global maxVelocity and
 * maxAcceleration; constraint zones and tracking lag are ignored. {@code wait} adds its time;
 * sequential adds, parallel ends at the longest child, race at the shortest, deadline at the first
 * child. A named command fires when its step starts; a path event marker fires when the ideal
 * trajectory reaches the marker position (PathPlanner fires markers on trajectory time).
 */
public final class PathPlannerAudit {
  private static final ObjectMapper MAPPER = new ObjectMapper();
  private static final int SAMPLES_PER_SEGMENT = 120;

  private final Path root;
  private final Map<String, PpPath> paths = new HashMap<>();

  /**
   * @param root the pathplanner directory (contains autos/ and paths/)
   */
  public PathPlannerAudit(Path root) {
    this.root = root;
  }

  /** every auto name (without extension) in the directory, sorted */
  public List<String> autoNames() throws IOException {
    try (Stream<Path> s = Files.list(root.resolve("autos"))) {
      return s.map(p -> p.getFileName().toString())
          .filter(n -> n.endsWith(".auto"))
          .map(n -> n.substring(0, n.length() - ".auto".length()))
          .sorted()
          .toList();
    }
  }

  public AutoTimeline timeline(String autoName) throws IOException {
    JsonNode auto = MAPPER.readTree(root.resolve("autos").resolve(autoName + ".auto").toFile());
    Builder b = new Builder(autoName);
    double end = b.run(auto.get("command"), 0.0);
    return new AutoTimeline(autoName, end, b.commands, b.positionFn());
  }

  /**
   * all paths named by the auto, in order of first use (for "was this auto's path file touched")
   */
  public List<String> pathNames(String autoName) throws IOException {
    JsonNode auto = MAPPER.readTree(root.resolve("autos").resolve(autoName + ".auto").toFile());
    List<String> out = new ArrayList<>();
    collectPaths(auto.get("command"), out);
    return out;
  }

  private static void collectPaths(JsonNode n, List<String> out) {
    if (n.get("type").asText().equals("path")) {
      String name = n.get("data").get("pathName").asText();
      if (!out.contains(name)) out.add(name);
    } else if (n.get("data").has("commands")) {
      for (JsonNode c : n.get("data").get("commands")) collectPaths(c, out);
    }
  }

  // ---------------------------------------------------------------------------------------------

  private PpPath path(String name) throws IOException {
    PpPath p = paths.get(name);
    if (p == null) {
      p = PpPath.load(root.resolve("paths").resolve(name + ".path").toFile());
      paths.put(name, p);
    }
    return p;
  }

  private final class Builder {
    final String autoName;
    final List<HoodCommand> commands = new ArrayList<>();
    final List<double[]> pieceStarts = new ArrayList<>(); // {t0, index into pieces}
    final List<PpPath> pieces = new ArrayList<>();

    Builder(String autoName) {
      this.autoName = autoName;
    }

    double run(JsonNode node, double t0) throws IOException {
      String type = node.get("type").asText();
      JsonNode data = node.get("data");
      switch (type) {
        case "wait":
          return t0 + data.get("waitTime").asDouble();
        case "named":
          addNamed(t0, data.get("name").asText(), "auto step");
          return t0;
        case "path":
          {
            String name = data.get("pathName").asText();
            PpPath p = path(name);
            pieceStarts.add(new double[] {t0, pieces.size()});
            pieces.add(p);
            if (p.markers != null) {
              for (JsonNode m : p.markers) {
                List<String> names = new ArrayList<>();
                namedIn(m.get("command"), names);
                double s = p.arcAtParam(m.get("waypointRelativePos").asDouble());
                for (String n : names) {
                  addNamed(
                      t0 + p.timeAtArc(s),
                      n,
                      String.format(
                          "marker %s@%.2f", name, m.get("waypointRelativePos").asDouble()));
                }
              }
            }
            return t0 + p.time;
          }
        case "sequential":
          {
            double t = t0;
            for (JsonNode c : data.get("commands")) t = run(c, t);
            return t;
          }
        case "parallel":
        case "race":
        case "deadline":
          {
            double max = t0, min = Double.POSITIVE_INFINITY, first = Double.NaN;
            for (JsonNode c : data.get("commands")) {
              double e = run(c, t0);
              max = Math.max(max, e);
              min = Math.min(min, e);
              if (Double.isNaN(first)) first = e;
            }
            return type.equals("parallel") ? max : type.equals("race") ? min : first;
          }
        default:
          throw new IllegalArgumentException(
              "unsupported PathPlanner command type '" + type + "' in auto " + autoName);
      }
    }

    void addNamed(double t, String name, String source) {
      HoodCommand c = HoodCommand.ofName(t, name, source);
      if (c != null) commands.add(c);
    }

    AutoTimeline.Position positionFn() {
      if (pieces.isEmpty()) return null;
      return t -> {
        // the last path that has started by time t
        int idx = 0;
        for (int i = 0; i < pieceStarts.size(); i++) {
          if (pieceStarts.get(i)[0] <= t) idx = i;
        }
        PpPath p = pieces.get(idx);
        double tau = Math.max(0.0, Math.min(p.time, t - pieceStarts.get(idx)[0]));
        return p.pointAtArc(p.arcAtTime(tau));
      };
    }
  }

  private static void namedIn(JsonNode command, List<String> out) {
    if (command == null || command.isNull()) return;
    if (command.get("type").asText().equals("named")) {
      out.add(command.get("data").get("name").asText());
    } else if (command.get("data").has("commands")) {
      for (JsonNode c : command.get("data").get("commands")) namedIn(c, out);
    }
  }

  // ---------------------------------------------------------------------------------------------

  /** A sampled PathPlanner path (cubic Beziers between waypoints) with a trapezoidal time law. */
  static final class PpPath {
    double[] param;
    double[] x;
    double[] y;
    double[] dist;
    double length;
    double vmax;
    double amax;
    double time;
    JsonNode markers;

    static PpPath load(File f) throws IOException {
      JsonNode j = MAPPER.readTree(f);
      JsonNode w = j.get("waypoints");
      int segs = w.size() - 1;
      int n = segs * SAMPLES_PER_SEGMENT + 1;
      PpPath p = new PpPath();
      p.param = new double[n];
      p.x = new double[n];
      p.y = new double[n];
      p.dist = new double[n];
      int k = 0;
      double dist = 0;
      for (int i = 0; i < segs; i++) {
        double[] p0 = pt(w.get(i).get("anchor"));
        double[] p1 = pt(w.get(i).get("nextControl"));
        double[] p2 = pt(w.get(i + 1).get("prevControl"));
        double[] p3 = pt(w.get(i + 1).get("anchor"));
        for (int s = (i == 0 ? 0 : 1); s <= SAMPLES_PER_SEGMENT; s++) {
          double t = s / (double) SAMPLES_PER_SEGMENT;
          double u = 1 - t;
          double px =
              u * u * u * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t * t * t * p3[0];
          double py =
              u * u * u * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t * t * t * p3[1];
          if (k > 0) dist += Math.hypot(px - p.x[k - 1], py - p.y[k - 1]);
          p.param[k] = i + t;
          p.x[k] = px;
          p.y[k] = py;
          p.dist[k] = dist;
          k++;
        }
      }
      p.length = dist;
      p.vmax = j.get("globalConstraints").get("maxVelocity").asDouble();
      p.amax = j.get("globalConstraints").get("maxAcceleration").asDouble();
      double dacc = p.vmax * p.vmax / (2 * p.amax);
      p.time =
          p.length >= 2 * dacc
              ? 2 * p.vmax / p.amax + (p.length - 2 * dacc) / p.vmax
              : 2 * Math.sqrt(p.length / p.amax);
      p.markers = j.get("eventMarkers");
      return p;
    }

    private static double[] pt(JsonNode n) {
      return new double[] {n.get("x").asDouble(), n.get("y").asDouble()};
    }

    /** arc length at a waypoint-relative position (waypoint index + Bezier t) */
    double arcAtParam(double pos) {
      int lo = 0, hi = param.length - 1;
      while (lo < hi) {
        int mid = (lo + hi) >>> 1;
        if (param[mid] < pos) lo = mid + 1;
        else hi = mid;
      }
      int i = lo;
      if (i > 0 && Math.abs(param[i - 1] - pos) < Math.abs(param[i] - pos)) i--;
      return dist[i];
    }

    double timeAtArc(double s) {
      s = Math.max(0.0, Math.min(length, s));
      double dacc = vmax * vmax / (2 * amax);
      if (length >= 2 * dacc) {
        if (s <= dacc) return Math.sqrt(2 * s / amax);
        if (s <= length - dacc) return vmax / amax + (s - dacc) / vmax;
        return time - Math.sqrt(2 * (length - s) / amax);
      }
      return s <= length / 2 ? Math.sqrt(2 * s / amax) : time - Math.sqrt(2 * (length - s) / amax);
    }

    double arcAtTime(double tau) {
      tau = Math.max(0.0, Math.min(time, tau));
      double dacc = vmax * vmax / (2 * amax);
      if (length >= 2 * dacc) {
        double ta = vmax / amax;
        if (tau <= ta) return 0.5 * amax * tau * tau;
        if (tau <= time - ta) return dacc + vmax * (tau - ta);
        return length - 0.5 * amax * (time - tau) * (time - tau);
      }
      return tau <= time / 2
          ? 0.5 * amax * tau * tau
          : length - 0.5 * amax * (time - tau) * (time - tau);
    }

    double[] pointAtArc(double s) {
      int lo = 0, hi = dist.length - 1;
      while (lo < hi) {
        int mid = (lo + hi) >>> 1;
        if (dist[mid] < s) lo = mid + 1;
        else hi = mid;
      }
      int i = lo;
      if (i > 0 && Math.abs(dist[i - 1] - s) < Math.abs(dist[i] - s)) i--;
      return new double[] {x[i], y[i]};
    }
  }
}
