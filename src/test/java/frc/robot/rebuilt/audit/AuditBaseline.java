package frc.robot.rebuilt.audit;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * The checked-in list of KNOWN hood/trench violations ({@code baseline.json}).
 *
 * <ul>
 *   <li>{@code static}: per kind ("pathplanner") and auto, the pass numbers (1-based
 *       trench stays) whose hood angle is above the down threshold in the static analysis
 *   <li>{@code staticGuarded}: the same, after applying the robot's trench guard ({@link
 *       GuardedTimeline}); this is what the robot is expected to do
 *   <li>{@code sim}: per kind, autos whose headless simulation shows the hood above {@link
 *       #SIM_THRESHOLD_DEG} under a trench beam
 * </ul>
 *
 * Tests fail on violations that are not listed (new problems). When an issue is fixed, regenerate
 * with {@code AUDIT_UPDATE_BASELINE=1} and commit the smaller file: the diff is the list of fixes.
 */
public final class AuditBaseline {
  /** Sim results are noisier than the static model, so only clear violations are tracked. */
  public static final double SIM_THRESHOLD_DEG = 22.0;

  public static final Path FILE = Path.of("src/test/resources/auto-audit/baseline.json");
  private static final ObjectMapper MAPPER =
      new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

  public final Map<String, Map<String, List<Integer>>> staticViolations = new TreeMap<>();
  public final Map<String, Map<String, List<Integer>>> guardedViolations = new TreeMap<>();
  public final Map<String, List<String>> simViolations = new TreeMap<>();

  public static AuditBaseline load() throws IOException {
    AuditBaseline b = new AuditBaseline();
    if (!Files.exists(FILE)) return b;
    JsonNode root = MAPPER.readTree(FILE.toFile());
    if (root.has("static")) {
      b.staticViolations.putAll(
          MAPPER.convertValue(
              root.get("static"),
              new TypeReference<TreeMap<String, TreeMap<String, List<Integer>>>>() {}));
    }
    if (root.has("staticGuarded")) {
      b.guardedViolations.putAll(
          MAPPER.convertValue(
              root.get("staticGuarded"),
              new TypeReference<TreeMap<String, TreeMap<String, List<Integer>>>>() {}));
    }
    if (root.has("sim")) {
      b.simViolations.putAll(
          MAPPER.convertValue(
              root.get("sim"), new TypeReference<TreeMap<String, List<String>>>() {}));
    }
    return b;
  }

  public void save() throws IOException {
    ObjectNode root = MAPPER.createObjectNode();
    root.put(
        "_about",
        "Known hood/trench violations. Tests fail on anything NOT listed here. Fixed an issue?"
            + " Regenerate with AUDIT_UPDATE_BASELINE=1 (see docs/AUTO_AUDIT.md) and commit the"
            + " diff.");
    root.set("static", MAPPER.valueToTree(staticViolations));
    root.set("staticGuarded", MAPPER.valueToTree(guardedViolations));
    root.set("sim", MAPPER.valueToTree(simViolations));
    Files.createDirectories(FILE.getParent());
    Files.writeString(FILE, MAPPER.writeValueAsString(root) + "\n");
  }

  public List<Integer> staticFor(String kind, String name) {
    return staticViolations.getOrDefault(kind, Map.of()).getOrDefault(name, List.of());
  }

  public List<Integer> guardedFor(String kind, String name) {
    return guardedViolations.getOrDefault(kind, Map.of()).getOrDefault(name, List.of());
  }

  public void setGuarded(String kind, String name, List<Integer> passes) {
    Map<String, List<Integer>> m = guardedViolations.computeIfAbsent(kind, k -> new TreeMap<>());
    if (passes.isEmpty()) m.remove(name);
    else m.put(name, new ArrayList<>(passes));
    if (m.isEmpty()) guardedViolations.remove(kind);
  }

  public boolean simKnown(String kind, String name) {
    return simViolations.getOrDefault(kind, List.of()).contains(name);
  }

  public void setStatic(String kind, String name, List<Integer> passes) {
    Map<String, List<Integer>> m = staticViolations.computeIfAbsent(kind, k -> new TreeMap<>());
    if (passes.isEmpty()) m.remove(name);
    else m.put(name, new ArrayList<>(passes));
    if (m.isEmpty()) staticViolations.remove(kind);
  }

  public void setSim(String kind, String name, boolean violating) {
    List<String> l = simViolations.computeIfAbsent(kind, k -> new ArrayList<>());
    l.remove(name);
    if (violating) {
      l.add(name);
      l.sort(String::compareTo);
    }
    if (l.isEmpty()) simViolations.remove(kind);
  }
}
