package frc.robot.rebuilt.audit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.DisabledIfEnvironmentVariable;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * Static hood-vs-trench audit of the REAL autos in {@code src/main/deploy}.
 *
 * <p>Fails when an auto has a trench pass with the hood above the down threshold that is not in
 * {@code baseline.json}. Run with {@code AUDIT_STRICT_BASELINE=1} to also fail when a listed issue
 * no longer reproduces (so fixed items must be removed), or {@code AUDIT_UPDATE_BASELINE=1} to
 * rewrite the static section. Always writes {@code build/auto-audit/report.md}.
 */
public class AutoAuditBaselineTest {
  static final Path PATHPLANNER = Path.of("src/main/deploy/pathplanner");
  static final Path BLINE = Path.of("src/main/deploy/autos");

  /** kind -> name -> stays (centre criterion) */
  static Map<String, Map<String, List<Stay>>> analyzeAll(
      Map<String, Map<String, AutoTimeline>> timelines) throws IOException {
    Map<String, Map<String, List<Stay>>> out = new TreeMap<>();
    PathPlannerAudit pp = new PathPlannerAudit(PATHPLANNER);
    for (String n : pp.autoNames()) {
      AutoTimeline tl = pp.timeline(n);
      timelines.computeIfAbsent("pathplanner", k -> new TreeMap<>()).put(n, tl);
      out.computeIfAbsent("pathplanner", k -> new TreeMap<>())
          .put(n, TrenchAudit.stays(tl, TrenchGeometry.CENTER));
    }
    BLineAudit bl = new BLineAudit(BLINE);
    for (String n : bl.pathNames()) {
      AutoTimeline tl = bl.timeline(n);
      timelines.computeIfAbsent("bline", k -> new TreeMap<>()).put(n, tl);
      out.computeIfAbsent("bline", k -> new TreeMap<>())
          .put(n, TrenchAudit.stays(tl, TrenchGeometry.CENTER));
    }
    return out;
  }

  @Test
  public void everyAutoIsAnalyzed() throws IOException {
    Map<String, Map<String, AutoTimeline>> tls = new TreeMap<>();
    Map<String, Map<String, List<Stay>>> all = analyzeAll(tls);
    assertTrue(
        all.get("pathplanner").size() >= 40,
        "pathplanner autos found: " + all.get("pathplanner").size());
    assertTrue(all.get("bline").size() >= 10, "bline paths found: " + all.get("bline").size());
  }

  @Test
  @DisabledIfEnvironmentVariable(named = "AUDIT_UPDATE_BASELINE", matches = "1")
  public void noNewHoodTrenchViolations() throws IOException {
    AuditBaseline baseline = AuditBaseline.load();
    Map<String, Map<String, AutoTimeline>> tls = new TreeMap<>();
    Map<String, Map<String, List<Stay>>> all = analyzeAll(tls);

    List<String> fresh = new ArrayList<>();
    List<String> stale = new ArrayList<>();
    for (var kind : all.entrySet()) {
      for (var auto : kind.getValue().entrySet()) {
        List<Integer> now = TrenchAudit.violationPasses(auto.getValue());
        List<Integer> known = baseline.staticFor(kind.getKey(), auto.getKey());
        for (int pass : now) {
          if (!known.contains(pass)) {
            Stay s = auto.getValue().get(pass - 1);
            fresh.add(String.format("%s '%s' pass %d: %s", kind.getKey(), auto.getKey(), pass, s));
          }
        }
        for (int pass : known) {
          if (!now.contains(pass)) {
            stale.add(String.format("%s '%s' pass %d", kind.getKey(), auto.getKey(), pass));
          }
        }
      }
    }
    // baseline entries for autos that no longer exist also count as fixed/removed
    for (var kind : baseline.staticViolations.entrySet()) {
      for (String auto : kind.getValue().keySet()) {
        if (!all.getOrDefault(kind.getKey(), Map.of()).containsKey(auto)) {
          stale.add(String.format("%s '%s' (auto no longer exists)", kind.getKey(), auto));
        }
      }
    }
    if (!stale.isEmpty()) {
      System.out.println(
          "[auto-audit] FIXED since baseline (remove with AUDIT_UPDATE_BASELINE=1):");
      stale.forEach(s -> System.out.println("  - " + s));
    }
    assertTrue(
        fresh.isEmpty(),
        "New hood-above-threshold trench passes (not in baseline.json):\n  "
            + String.join("\n  ", fresh)
            + "\n"
            + "Fix the auto (lower the hood earlier / raise it later), or if this is accepted run"
            + " the tests with AUDIT_UPDATE_BASELINE=1.");
    if ("1".equals(System.getenv("AUDIT_STRICT_BASELINE"))) {
      assertTrue(
          stale.isEmpty(), "Fixed but still in baseline.json:\n  " + String.join("\n  ", stale));
    }
  }

  @Test
  public void writesReport() throws IOException {
    AuditBaseline baseline = AuditBaseline.load();
    Map<String, Map<String, AutoTimeline>> tls = new TreeMap<>();
    Map<String, Map<String, List<Stay>>> all = analyzeAll(tls);
    StringBuilder sb = new StringBuilder("# Hood / trench static audit (generated)\n\n");
    sb.append("Hood must be <= ")
        .append(HoodModel.DOWN_THRESHOLD_DEG)
        .append(" deg under a trench beam.\n\n");
    for (var kind : all.entrySet()) {
      sb.append("## ").append(kind.getKey()).append("\n\n");
      for (var auto : kind.getValue().entrySet()) {
        List<Stay> stays = auto.getValue();
        AutoTimeline tl = tls.get(kind.getKey()).get(auto.getKey());
        List<Integer> bad = TrenchAudit.violationPasses(stays);
        sb.append(
            String.format(
                "- [%s] %s (%d trench passes, lowers after raise: %s)\n",
                bad.isEmpty() ? "x" : " ", auto.getKey(), stays.size(), tl.lowerAfterRaise()));
        for (int pass : bad) {
          Stay s = stays.get(pass - 1);
          boolean known = baseline.staticFor(kind.getKey(), auto.getKey()).contains(pass);
          sb.append(String.format("    - pass %d: %s%s\n", pass, s, known ? "" : "  **NEW**"));
        }
      }
      sb.append('\n');
    }
    Path out = Path.of("build/auto-audit/report.md");
    Files.createDirectories(out.getParent());
    Files.writeString(out, sb.toString());
    assertTrue(Files.size(out) > 200);
  }

  @Test
  @EnabledIfEnvironmentVariable(named = "AUDIT_UPDATE_BASELINE", matches = "1")
  public void updateBaseline() throws IOException {
    AuditBaseline baseline = AuditBaseline.load();
    baseline.staticViolations.clear();
    Map<String, Map<String, AutoTimeline>> tls = new TreeMap<>();
    for (var kind : analyzeAll(tls).entrySet()) {
      for (var auto : kind.getValue().entrySet()) {
        baseline.setStatic(
            kind.getKey(), auto.getKey(), TrenchAudit.violationPasses(auto.getValue()));
      }
    }
    baseline.save();
    assertEquals(
        baseline.staticViolations.isEmpty(),
        false,
        "no violations left? then delete the static baseline");
  }
}
