// ============================================================================
//  test_core.cpp -- B stage judge (written by me, DO NOT EDIT)
//
//  ASCII only on purpose: the Windows console here is GBK, so UTF-8 Chinese
//  printed by this program would show up as mojibake.
//
//  Run with:  .\tools\check_core.ps1
// ============================================================================

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "AngleTrack.h"
#include "FloatFrame.h"
#include "Pid.h"
#include "SafetyGuard.h"
#include "SineRef.h"
#include "WatchdogPolicy.h"

namespace {

int g_pass = 0;
int g_fail = 0;

void ok(const char* name) {
  std::printf("[ ok ] %s\n", name);
  ++g_pass;
}

void fail(const char* name, const char* detail) {
  std::printf("[FAIL] %s  (%s)\n", name, detail);
  ++g_fail;
}

void check_near(const char* name, float got, float want, float tol) {
  if (std::fabs(got - want) <= tol) {
    ok(name);
    return;
  }
  char buf[160];
  std::snprintf(buf, sizeof(buf), "got %.5f, want %.5f (tol %.5f)", got, want, tol);
  fail(name, buf);
}

void check_int(const char* name, long long got, long long want) {
  if (got == want) {
    ok(name);
    return;
  }
  char buf[160];
  std::snprintf(buf, sizeof(buf), "got %lld, want %lld", got, want);
  fail(name, buf);
}

void check_true(const char* name, bool cond, const char* detail) {
  if (cond) {
    ok(name);
    return;
  }
  fail(name, detail);
}

// byte-wise comparison (used by B6); a null buffer is a failure, not a crash
void check_bytes(const char* name, const uint8_t* got, const uint8_t* want, unsigned n) {
  if (got == nullptr) {
    fail(name, "buffer pointer is null");
    return;
  }
  if (std::memcmp(got, want, n) == 0) {
    ok(name);
    return;
  }
  char buf[220];
  int off = std::snprintf(buf, sizeof(buf), "got");
  for (unsigned i = 0; i < n && off > 0 && off < static_cast<int>(sizeof(buf)) - 8; ++i) {
    off += std::snprintf(buf + off, sizeof(buf) - static_cast<size_t>(off), " %02X", got[i]);
  }
  if (off > 0 && off < static_cast<int>(sizeof(buf)) - 8) {
    off += std::snprintf(buf + off, sizeof(buf) - static_cast<size_t>(off), "  want");
    for (unsigned i = 0; i < n && off > 0 && off < static_cast<int>(sizeof(buf)) - 8; ++i) {
      off += std::snprintf(buf + off, sizeof(buf) - static_cast<size_t>(off), " %02X", want[i]);
    }
  }
  fail(name, buf);
}

constexpr float kTol = 1e-3f;  // float comparison tolerance

// run n ticks, return the value of the last tick
float run(rm::SineRef& s, unsigned n) {
  float v = 0.0f;
  for (unsigned i = 0; i < n; ++i) {
    v = s.step();
  }
  return v;
}

}  // namespace

int main() {
  std::printf("---- B1  sine reference generator (SineRef) ----\n");

  // 1. initial state
  {
    rm::SineRef s;
    check_near("initial value() is 0", s.value(), 0.0f, kTol);
    check_int("initial ticks() is 0", static_cast<long long>(s.ticks()), 0);
  }

  // 2. A=1, f=1Hz, phase 0: four key points of one period
  {
    rm::SineRef s;
    s.configure(1.0f, 1.0f, 0.0f);
    check_near("f=1Hz: tick 250 (0.25s) is +1", run(s, 250), 1.0f, kTol);
    check_near("f=1Hz: tick 500 (0.50s) is 0", run(s, 250), 0.0f, kTol);
    check_near("f=1Hz: tick 750 (0.75s) is -1", run(s, 250), -1.0f, kTol);
    check_near("f=1Hz: tick 1000 (1.0s) is 0", run(s, 250), 0.0f, kTol);
    check_int("ticks() == 1000 after one period", static_cast<long long>(s.ticks()), 1000);
  }

  // 3. half period symmetry: v(t + T/2) == -v(t)
  {
    rm::SineRef a;
    rm::SineRef b;
    a.configure(1.3f, 1.0f, 0.0f);
    b.configure(1.3f, 1.0f, 0.0f);
    run(a, 125);  // t = 0.125s -> 1.3*sin(pi/4)  = +0.919
    run(b, 625);  // t = 0.625s -> 1.3*sin(5pi/4) = -0.919
    check_near("half-period symmetry: v(0.625s) == -v(0.125s)", b.value(), -a.value(), kTol);
  }

  // 4. amplitude is respected and reached
  {
    rm::SineRef s;
    s.configure(2.0f, 1.0f, 0.0f);
    float max_v = 0.0f;
    for (unsigned i = 0; i < 1000; ++i) {
      const float v = s.step();
      if (std::fabs(v) > max_v) {
        max_v = std::fabs(v);
      }
    }
    check_true("amplitude 2: never exceeds 2", max_v <= 2.0f + kTol, "value went above amplitude");
    check_near("amplitude 2: peak reaches 2", max_v, 2.0f, kTol);
  }

  // 5. initial phase: with phi = pi/2 the first tick is already near the peak
  {
    rm::SineRef s;
    s.configure(1.0f, 1.0f, 1.57079633f);
    check_near("phase pi/2: first tick is ~+1", run(s, 1), 1.0f, 5e-3f);
  }

  // 6. reset() brings both time and value back to zero
  {
    rm::SineRef s;
    s.configure(1.0f, 1.0f, 0.0f);
    run(s, 300);
    s.reset();
    check_int("reset(): ticks() back to 0", static_cast<long long>(s.ticks()), 0);
    check_near("reset(): value() back to 0", s.value(), 0.0f, kTol);
    check_near("reset(): tick 250 is +1 again", run(s, 250), 1.0f, kTol);
  }

  // 7. value() agrees with the last step() return
  {
    rm::SineRef s;
    s.configure(1.0f, 0.7f, 0.3f);
    const float v = run(s, 123);
    check_near("value() equals last step() return", s.value(), v, 1e-6f);
    check_int("ticks() == 123 after 123 steps", static_cast<long long>(s.ticks()), 123);
  }

  // 8. zero amplitude means always zero
  {
    rm::SineRef s;
    s.configure(0.0f, 1.0f, 0.0f);
    float max_v = 0.0f;
    for (unsigned i = 0; i < 500; ++i) {
      const float v = s.step();
      if (std::fabs(v) > max_v) {
        max_v = std::fabs(v);
      }
    }
    check_near("amplitude 0: always 0", max_v, 0.0f, kTol);
  }

  // 9. low frequency curve: f=0.2Hz after 1s must be A*sin(2*pi*0.2)
  {
    rm::SineRef s;
    s.configure(3.0f, 0.2f, 0.0f);
    const float want = 3.0f * std::sin(rm::kTwoPi * 0.2f * 1.0f);
    check_near("f=0.2Hz: after 1s equals 3*sin(2*pi*0.2)", run(s, 1000), want, kTol);
  }

  // 10. parameters can be changed at runtime
  {
    rm::SineRef s;
    s.configure(1.0f, 1.0f, 0.0f);
    run(s, 250);
    s.configure(0.0f, 1.0f, 0.0f);
    check_near("set amplitude 0: output is 0 immediately", run(s, 1), 0.0f, kTol);
  }

  // ======================= B2: PID controller =======================
  std::printf("\n---- B2  PID controller (Pid) ----\n");

  // 11. zero error -> zero output
  {
    rm::Pid p(1.0f, 0.0f, 0.0f, 3000.0f, -3000.0f);
    check_near("PID: zero error -> zero output", p.calc(0.0f, 0.0f), 0.0f, kTol);
  }

  // 12. P term: proportional to the error, and has no memory
  {
    rm::Pid p(1.0f, 0.0f, 0.0f, 3000.0f, -3000.0f);
    check_near("P only: err=100, Kp=1 -> 100", p.calc(100.0f, 0.0f), 100.0f, kTol);
    bool steady = true;
    for (int i = 0; i < 10; ++i) {
      if (std::fabs(p.calc(100.0f, 0.0f) - 100.0f) > kTol) {
        steady = false;
      }
    }
    check_true("P only: stays 100 over 10 calls (no memory)", steady, "P term drifted");
  }

  // 13. I term: integrates the error over time  (Ki * sum(e) * T)
  {
    rm::Pid p(0.0f, 1.0f, 0.0f, 10000.0f, -10000.0f);
    check_near("I only: 1st call Ki=1 err=100 -> 0.1", p.calc(100.0f, 0.0f), 0.1f, kTol);
    for (int i = 0; i < 8; ++i) {
      p.calc(100.0f, 0.0f);
    }
    check_near("I only: 10th call -> 1.0 (linear ramp)", p.calc(100.0f, 0.0f), 1.0f, kTol);
  }

  // 14. D term: reacts to how fast the error changes; steady error -> 0
  {
    rm::Pid p(0.0f, 0.0f, 0.001f, 10000.0f, -10000.0f);
    p.calc(100.0f, 0.0f);  // first call: derivative behaviour is not specified
    check_near("D only: steady error -> 0", p.calc(100.0f, 0.0f), 0.0f, kTol);
    // err goes 100 -> -100, so deriv = -200 and out = Kd * deriv / T = -200
    check_near("D only: err falls by 200 -> -200", p.calc(0.0f, 100.0f), -200.0f, kTol);
  }

  // 15. output clamping
  {
    rm::Pid p(1000.0f, 0.0f, 0.0f, 3000.0f, -3000.0f);
    check_near("clamp: raw +100000 -> +3000", p.calc(100.0f, 0.0f), 3000.0f, kTol);
    check_near("clamp: raw -100000 -> -3000", p.calc(0.0f, 100.0f), -3000.0f, kTol);
  }

  // 16. anti-windup: the integral must not keep growing while the output is saturated
  {
    rm::Pid p(0.0f, 10000.0f, 0.0f, 1000.0f, -1000.0f);
    float u = 0.0f;
    for (int i = 0; i < 50; ++i) {
      u = p.calc(100.0f, 0.0f);
    }
    check_near("anti-windup: holds +limit while saturated", u, 1000.0f, kTol);
    // Without rollback the integral would keep growing, and after the error reverses
    // the output would stay pinned at +1000 for a long time. So the real check is:
    // does the output react to the reversed error immediately? (exact value not fixed)
    const float u_rev = p.calc(0.0f, 100.0f);
    check_true("anti-windup: error reversal is not delayed", u_rev <= 1.0f,
               "output stuck at the positive limit");
  }

  // 17. reset() clears the integral
  {
    rm::Pid p(0.0f, 1000.0f, 0.0f, 10000.0f, -10000.0f);
    for (int i = 0; i < 10; ++i) {
      p.calc(100.0f, 0.0f);
    }
    p.reset();
    check_near("reset(): integral cleared", p.calc(100.0f, 0.0f), 100.0f, kTol);
  }

  // 18. closed loop on a first order plant: velocity loop must converge
  {
    rm::Pid p(2.0f, 8.0f, 0.002f, 25000.0f, -25000.0f);
    float v = 0.0f;
    const float tau = 0.05f;
    for (int i = 0; i < 2000; ++i) {
      const float u = p.calc(3.0f, v);
      v += (u - v) * (rm::kSampleTime / tau);
    }
    check_near("closed loop: velocity converges to 3 rad/s", v, 3.0f, 0.06f);
  }

  // 19. sine tracking: the real assessment scenario, measured over the last period
  {
    rm::Pid p(2.0f, 8.0f, 0.002f, 25000.0f, -25000.0f);
    rm::SineRef ref;
    ref.configure(2.0f, 0.2f, 0.0f);  // 2 rad/s amplitude, 0.2 Hz
    float v = 0.0f;
    float peak_target = 0.0f;
    float peak_v = 0.0f;
    const float tau = 0.05f;
    for (int i = 0; i < 15000; ++i) {
      const float target = ref.step();
      const float u = p.calc(target, v);
      v += (u - v) * (rm::kSampleTime / tau);
      if (i >= 10000) {  // last full period (5 s at 0.2 Hz)
        if (std::fabs(target) > peak_target) {
          peak_target = std::fabs(target);
        }
        if (std::fabs(v) > peak_v) {
          peak_v = std::fabs(v);
        }
      }
    }
    check_true("sine tracking: target peak recorded", peak_target > 1.9f, "sine target looks wrong");
    check_true("sine tracking: velocity keeps 80% of the amplitude",
               peak_v >= 0.8f * peak_target, "too much amplitude loss");
  }

  // ======================= B3: multi-turn angle =======================
  std::printf("\n---- B3  multi-turn angle (AngleTrack) ----\n");

  const float tick = rm::kRawToRad;  // one raw tick in radians

  // 20. reset() -> clean state
  {
    rm::AngleTrack a;
    a.reset();
    check_near("reset(): rad() is 0", a.rad(), 0.0f, kTol);
    check_near("reset(): turns() is 0", a.turns(), 0.0f, kTol);
    check_int("reset(): totalTicks() is 0", a.totalTicks(), 0);
  }

  // 21. the first frame is only the start point (no fake turn)
  {
    rm::AngleTrack a;
    a.reset();
    a.update(5000);
    check_near("first frame at raw 5000: no fake turn", a.rad(), 0.0f, kTol);
    check_int("first frame at raw 5000: totalTicks() stays 0", a.totalTicks(), 0);
  }
  {
    rm::AngleTrack a;
    a.reset();
    a.update(8191);
    check_near("first frame at raw 8191: no fake turn", a.rad(), 0.0f, kTol);
  }

  // 22. small moves inside one turn
  {
    rm::AngleTrack a;
    a.reset();
    a.update(4000);
    a.update(4100);
    check_near("+100 ticks -> +100 * kRawToRad", a.rad(), 100.0f * tick, 1e-4f);
  }
  {
    rm::AngleTrack a;
    a.reset();
    a.update(4000);
    a.update(3900);
    check_near("-100 ticks -> -100 * kRawToRad", a.rad(), -100.0f * tick, 1e-4f);
  }

  // 23. crossing zero, forward: raw falls by ~8191 but we actually moved +1 tick
  {
    rm::AngleTrack a;
    a.reset();
    a.update(8191);
    a.update(0);
    check_near("wrap fwd: 8191 -> 0 is +1 tick", a.rad(), 1.0f * tick, 1e-4f);
    check_int("wrap fwd: 8191 -> 0 totalTicks is +1", a.totalTicks(), 1);
  }
  {
    rm::AngleTrack a;
    a.reset();
    a.update(8180);
    a.update(10);
    check_int("wrap fwd: 8180 -> 10 is +22 ticks", a.totalTicks(), 22);
  }

  // 24. crossing zero, backward: raw rises by ~8191 but we actually moved -1 tick
  {
    rm::AngleTrack a;
    a.reset();
    a.update(0);
    a.update(8191);
    check_near("wrap bwd: 0 -> 8191 is -1 tick", a.rad(), -1.0f * tick, 1e-4f);
    check_int("wrap bwd: 0 -> 8191 totalTicks is -1", a.totalTicks(), -1);
  }
  {
    rm::AngleTrack a;
    a.reset();
    a.update(10);
    a.update(8170);
    check_int("wrap bwd: 10 -> 8170 is -32 ticks", a.totalTicks(), -32);
  }

  // 25. jitter around the boundary must NOT pile up turns
  {
    rm::AngleTrack a;
    a.reset();
    a.update(8191);
    a.update(0);     // +1
    a.update(8191);  // -1
    a.update(0);     // +1
    a.update(8191);  // -1
    check_int("boundary jitter 4 moves nets 0 ticks", a.totalTicks(), 0);
  }

  // 26. many turns forward: 200 steps of +100 ticks = 20000 ticks
  {
    rm::AngleTrack a;
    a.reset();
    int32_t cur = 0;
    a.update(static_cast<uint16_t>(cur));
    for (int i = 0; i < 200; ++i) {
      cur += 100;
      if (cur >= rm::kRawPerTurn) {
        cur -= rm::kRawPerTurn;
      }
      a.update(static_cast<uint16_t>(cur));
    }
    check_int("20000 ticks forward: totalTicks is 20000", a.totalTicks(), 20000);
    check_near("20000 ticks forward: rad() matches", a.rad(), 20000.0f * tick, 1e-3f);
    check_near("20000 ticks forward: turns() is ~2.44", a.turns(), 2.4414f, 1e-3f);
  }

  // 27. forward then backward returns to where we started
  {
    rm::AngleTrack a;
    a.reset();
    int32_t cur = 5000;
    a.update(static_cast<uint16_t>(cur));
    for (int i = 0; i < 100; ++i) {          // 100 * +100 = +10000 ticks
      cur += 100;
      if (cur >= rm::kRawPerTurn) {
        cur -= rm::kRawPerTurn;
      }
      a.update(static_cast<uint16_t>(cur));
    }
    for (int i = 0; i < 100; ++i) {          // back: -10000 ticks
      cur -= 100;
      if (cur < 0) {
        cur += rm::kRawPerTurn;
      }
      a.update(static_cast<uint16_t>(cur));
    }
    check_int("out and back: totalTicks is 0", a.totalTicks(), 0);
    check_near("out and back: rad() is 0", a.rad(), 0.0f, 1e-3f);
  }

  // 28. one full turn backwards
  {
    rm::AngleTrack a;
    a.reset();
    int32_t cur = 4000;
    a.update(static_cast<uint16_t>(cur));
    for (int i = 0; i < 82; ++i) {           // 82 * -100 = -8200 ticks
      cur -= 100;
      if (cur < 0) {
        cur += rm::kRawPerTurn;
      }
      a.update(static_cast<uint16_t>(cur));
    }
    check_int("backwards one turn: totalTicks is -8200", a.totalTicks(), -8200);
    check_near("backwards one turn: turns() is ~-1", a.turns(), -1.0010f, 2e-3f);
  }

  // 29. documented assumption: a single frame never moves more than half a turn
  {
    rm::AngleTrack a;
    a.reset();
    a.update(1000);
    a.update(6000);  // looks like +5000, but a real frame cannot move that far,
                     // so it is read as -3192 (i.e. it went backwards over zero)
    check_int("+5000 jump is read as -3192 (single frame < half turn)", a.totalTicks(), -3192);
  }

  // 30. reset() re-arms the start point
  {
    rm::AngleTrack a;
    a.reset();
    a.update(8191);
    a.update(0);
    a.update(100);
    a.reset();
    a.update(7000);
    check_near("reset() re-arms: new first frame is 0 again", a.rad(), 0.0f, kTol);
    a.update(7100);
    check_near("reset() re-arms: next move counts from the new start", a.rad(), 100.0f * tick, 1e-4f);
  }

  // 31. the point of the whole level: the angle must not jump at the wrap
  {
    rm::AngleTrack a;
    a.reset();
    a.update(8100);
    const float r1 = a.rad();
    a.update(8191);
    const float r2 = a.rad();
    a.update(0);
    const float r3 = a.rad();
    a.update(100);
    const float r4 = a.rad();
    check_true("wrap continuity: angle keeps rising across 8191 -> 0",
               (r2 > r1) && (r3 > r2) && (r4 > r3), "angle jumped at the wrap");
  }

  // ======================= B4: safety guard =======================
  std::printf("\n---- B4  safety guard (SafetyGuard) ----\n");

  // 32. no output before the first feedback frame (power-on is the dangerous moment)
  {
    rm::SafetyGuard g;
    g.reset();
    g.tick(25.0f);
    check_true("no feedback yet: allowed() is false", !g.allowed(), "guard allowed output too early");
    check_near("no feedback yet: limit(1000) is 0", g.limit(1000.0f), 0.0f, kTol);
    check_true("no feedback yet: feedbackOk() is false", !g.feedbackOk(), "feedbackOk true too early");
  }

  // 33. one feedback frame -> output allowed
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    g.tick(25.0f);
    check_near("after one feedback: limit(1000) is 1000", g.limit(1000.0f), 1000.0f, kTol);
    check_true("after one feedback: feedbackOk() is true", g.feedbackOk(), "feedbackOk false");
  }

  // 34. feedback timeout: 20 ticks are fine, the 21st means we lost the motor
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    for (int i = 0; i < 20; ++i) {
      g.tick(25.0f);
    }
    check_near("feedback 20 ticks old: still allowed", g.limit(1000.0f), 1000.0f, kTol);
    g.tick(25.0f);  // 21st tick
    check_near("feedback 21 ticks old: output is 0", g.limit(1000.0f), 0.0f, kTol);
    check_true("feedback 21 ticks old: feedbackOk() is false", !g.feedbackOk(), "timeout not detected");
  }

  // 35. the link comes back -> output comes back by itself
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    for (int i = 0; i < 100; ++i) {
      g.tick(25.0f);
    }
    check_near("link lost for 100 ms: output stays 0", g.limit(-2000.0f), 0.0f, kTol);
    g.onFeedback();
    g.tick(25.0f);
    check_near("link back: output returns", g.limit(-2000.0f), -2000.0f, kTol);
  }

  // 36. over-temperature stops the output
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    g.tick(80.0f);
    check_true("over-temp 80C: tempOk() is false", !g.tempOk(), "tempOk true at 80C");
    check_near("over-temp 80C: output is 0", g.limit(1000.0f), 0.0f, kTol);
  }

  // 37. hysteresis: 70C trips, but it takes below 65C to come back
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    g.tick(80.0f);
    g.tick(68.0f);  // inside the dead band -> stay protected
    check_near("over-temp dead band 68C: still protected", g.limit(1000.0f), 0.0f, kTol);
    g.tick(69.9f);  // still inside
    check_near("over-temp dead band 69.9C: still protected", g.limit(1000.0f), 0.0f, kTol);
    g.tick(64.0f);  // below 70 - 5 -> release
    check_near("over-temp 64C: released, output returns", g.limit(1000.0f), 1000.0f, kTol);
  }

  // 38. absolute voltage cap (second line of defence)
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    g.tick(25.0f);
    check_near("safe cap: +99999 -> +3000", g.limit(99999.0f), 3000.0f, kTol);
    check_near("safe cap: -99999 -> -3000", g.limit(-99999.0f), -3000.0f, kTol);
    check_near("safe cap: +2000 passes unchanged", g.limit(2000.0f), 2000.0f, kTol);
    check_near("safe cap: -500 passes unchanged", g.limit(-500.0f), -500.0f, kTol);
  }

  // 39. reset() puts the guard back into the "power-on, no data" state
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    g.tick(25.0f);
    check_near("before reset: output allowed", g.limit(1000.0f), 1000.0f, kTol);
    g.reset();
    g.tick(25.0f);
    check_near("after reset: output blocked again", g.limit(1000.0f), 0.0f, kTol);
  }

  // 40. a cold motor with a dead link is still blocked (the two rules are independent)
  {
    rm::SafetyGuard g;
    g.reset();
    for (int i = 0; i < 50; ++i) {
      g.tick(20.0f);  // nice and cold, but no feedback ever arrived
    }
    check_near("cold but silent motor: still 0", g.limit(1000.0f), 0.0f, kTol);
  }

  // 41. over-temp is not the same as a lost link: after cooling down with frames
  //     still arriving, the motor is allowed again
  {
    rm::SafetyGuard g;
    g.reset();
    g.onFeedback();
    g.tick(80.0f);
    check_near("hot: blocked", g.limit(1000.0f), 0.0f, kTol);
    for (int i = 0; i < 3; ++i) {
      g.onFeedback();
      g.tick(60.0f);
    }
    check_near("cooled down with live link: allowed again", g.limit(1000.0f), 1000.0f, kTol);
  }

  // ======================= B5: watchdog feed policy =======================
  std::printf("\n---- B5  watchdog feed policy (WatchdogPolicy) ----\n");

  // 42. power-on: nothing has reported yet, so feeding is forbidden
  {
    rm::WatchdogPolicy w;
    w.reset();
    check_true("power-on: shouldFeed(0) is false", !w.shouldFeed(0), "fed before anything ran");
    check_true("power-on: controlOk is false", !w.controlOk(0), "controlOk too early");
    check_true("power-on: feedbackOk is false", !w.feedbackOk(0), "feedbackOk too early");
    check_true("power-on: loopOk is false", !w.loopOk(0), "loopOk too early");
  }

  // 43. all three must have reported before feeding is allowed
  {
    rm::WatchdogPolicy w;
    w.reset();
    w.onControlTick(100);
    check_true("only control tick: no feed yet", !w.shouldFeed(100), "fed too early");
    w.onFeedback(100);
    check_true("control + feedback: still no feed (main loop silent)", !w.shouldFeed(100), "fed too early");
    w.onMainLoop(100);
    check_true("all three healthy: feed allowed", w.shouldFeed(100), "feeding not allowed");
  }

  // 44. the 1 kHz control task stops -> stop feeding
  {
    rm::WatchdogPolicy w;
    w.reset();
    w.onControlTick(100);
    w.onFeedback(100);
    w.onMainLoop(100);
    check_true("control 5 ms old: still ok", w.shouldFeed(105), "control timeout too tight");
    check_true("control 6 ms old: no feed", !w.shouldFeed(106), "control stall not detected");
    w.onControlTick(110);
    check_true("control back: feeding resumes", w.shouldFeed(110), "control recovery failed");
  }

  // 45. the CAN link goes quiet -> stop feeding
  {
    rm::WatchdogPolicy w;
    w.reset();
    w.onControlTick(100);
    w.onFeedback(100);
    w.onMainLoop(100);
    w.onControlTick(121);
    w.onMainLoop(121);
    check_true("feedback 21 ms old: no feed", !w.shouldFeed(121), "stale CAN link not detected");
    w.onFeedback(122);
    check_true("feedback back: feeding resumes", w.shouldFeed(122), "feedback recovery failed");
  }

  // 46. the main loop blocks (e.g. a stuck serial print) -> stop feeding
  {
    rm::WatchdogPolicy w;
    w.reset();
    w.onControlTick(100);
    w.onFeedback(100);
    w.onMainLoop(100);
    w.onControlTick(301);
    w.onFeedback(301);
    check_true("main loop 201 ms old: no feed", !w.shouldFeed(301), "stuck main loop not detected");
    w.onMainLoop(302);
    check_true("main loop back: feeding resumes", w.shouldFeed(302), "loop recovery failed");
  }

  // 47. shouldFeed() is only a question: asking must not refresh anything
  {
    rm::WatchdogPolicy w;
    w.reset();
    w.onControlTick(100);
    w.onFeedback(100);
    w.onMainLoop(100);
    check_true("healthy at 100 ms", w.shouldFeed(100), "feeding not allowed");
    check_true("asking again at 106 ms does not refresh", !w.shouldFeed(106),
               "shouldFeed() had a side effect");
  }

  // 48. reset() -> back to the power-on state
  {
    rm::WatchdogPolicy w;
    w.reset();
    w.onControlTick(100);
    w.onFeedback(100);
    w.onMainLoop(100);
    check_true("before reset: allowed", w.shouldFeed(100), "feeding not allowed");
    w.reset();
    check_true("after reset: forbidden again", !w.shouldFeed(100), "reset did not clear the state");
  }

  // 49. the millisecond counter wraps: unsigned subtraction must stay correct
  {
    rm::WatchdogPolicy w;
    w.reset();
    w.onControlTick(0xFFFFFFF9u);
    w.onFeedback(0xFFFFFFF9u);
    w.onMainLoop(0xFFFFFFF9u);
    check_true("wrap: 5 ms later across the wrap is still healthy",
               w.shouldFeed(0xFFFFFFFEu), "wrap broke the timing");
    check_true("wrap: 7 ms later the control task is stale",
               !w.shouldFeed(0x00000000u), "wrap broke the timeout");
  }

  // ======================= B6: serial float frame =======================
  std::printf("\n---- B6  serial float frame (FloatFrame) ----\n");

  const uint8_t kTailBytes[4] = {0x00, 0x00, 0x80, 0x7F};

  // 50. a fresh frame holds nothing but the tail
  {
    rm::FloatFrame f;
    f.reset();
    check_int("fresh: count() is 0", f.count(), 0);
    check_int("fresh: size() is 4 (tail only)", f.size(), 4);
    check_true("fresh: empty() is true", f.empty(), "empty() lied");
    check_bytes("fresh: first 4 bytes are the tail", f.data(), kTailBytes, 4);
  }

  // 51. one float: little-endian payload, tail kept at the very end
  {
    rm::FloatFrame f;
    f.reset();
    check_true("push(1.0) returns true", f.push(1.0f), "push refused a plain value");
    check_int("push(1.0): count() is 1", f.count(), 1);
    check_int("push(1.0): size() is 8", f.size(), 8);
    check_true("push(1.0): empty() is false", !f.empty(), "empty() lied");
    const uint8_t want[8] = {0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x80, 0x7F};
    check_bytes("push(1.0): payload 00 00 80 3F + tail", f.data(), want, 8);
  }

  // 52. byte order is little-endian (low byte first)
  {
    rm::FloatFrame f;
    f.reset();
    f.push(-1.0f);
    const uint8_t want[8] = {0x00, 0x00, 0x80, 0xBF, 0x00, 0x00, 0x80, 0x7F};
    check_bytes("push(-1.0): little-endian 00 00 80 BF", f.data(), want, 8);
  }
  {
    rm::FloatFrame f;
    f.reset();
    f.push(0.5f);
    const uint8_t want[8] = {0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x80, 0x7F};
    check_bytes("push(0.5): little-endian 00 00 00 3F", f.data(), want, 8);
  }
  {
    rm::FloatFrame f;
    f.reset();
    f.push(2.5f);
    const uint8_t want[8] = {0x00, 0x00, 0x20, 0x40, 0x00, 0x00, 0x80, 0x7F};
    check_bytes("push(2.5): little-endian 00 00 20 40", f.data(), want, 8);
  }

  // 53. zero is a legal value and must not disturb the tail
  {
    rm::FloatFrame f;
    f.reset();
    f.push(0.0f);
    const uint8_t want[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7F};
    check_bytes("push(0.0): payload 00 00 00 00 + tail", f.data(), want, 8);
    check_int("push(0.0): size() is still 8", f.size(), 8);
  }

  // 54. a non-round value: 3.14159f == 0x40490FD0
  {
    rm::FloatFrame f;
    f.reset();
    f.push(3.14159f);
    const uint8_t want[8] = {0xD0, 0x0F, 0x49, 0x40, 0x00, 0x00, 0x80, 0x7F};
    check_bytes("push(3.14159): 0x40490FD0 little-endian", f.data(), want, 8);
  }

  // 55. three channels - the shape used for the sine-tracking plot
  {
    rm::FloatFrame f;
    f.reset();
    f.push(1.0f);
    f.push(-2.0f);
    f.push(0.25f);
    const uint8_t want[16] = {0x00, 0x00, 0x80, 0x3F,
                              0x00, 0x00, 0x00, 0xC0,
                              0x00, 0x00, 0x80, 0x3E,
                              0x00, 0x00, 0x80, 0x7F};
    check_bytes("3 channels: payload + one tail at the end", f.data(), want, 16);
    check_int("3 channels: count() is 3", f.count(), 3);
    check_int("3 channels: size() is 16", f.size(), 16);
  }

  // 56. size() grows by exactly 4 per float: size == count*4 + 4
  {
    rm::FloatFrame f;
    f.reset();
    bool shapeOk = true;
    for (unsigned n = 0; n <= 5; ++n) {
      if (f.size() != n * 4 + 4 || f.count() != n) {
        shapeOk = false;
      }
      f.push(static_cast<float>(n));
    }
    check_true("size() == count()*4 + 4 while filling", shapeOk, "size/count disagree");
  }

  // 57. capacity: kMaxFloats (15) fit, the next one is refused
  {
    rm::FloatFrame f;
    f.reset();
    bool allAccepted = true;
    for (unsigned i = 0; i < rm::FloatFrame::kMaxFloats; ++i) {
      if (!f.push(static_cast<float>(i))) {
        allAccepted = false;
      }
    }
    check_true("15 floats fit (kMaxFloats)", allAccepted, "a legal push was refused");
    check_int("full: count() is kMaxFloats", f.count(), rm::FloatFrame::kMaxFloats);
    check_int("full: size() is exactly kCapacity", f.size(), rm::FloatFrame::kCapacity);
    check_true("full: the 16th push is refused", !f.push(99.0f), "overflow was accepted");
    check_int("full: count() unchanged after refusal", f.count(), rm::FloatFrame::kMaxFloats);
    const uint8_t* p = f.data();
    if (p != nullptr && f.size() >= 4) {
      check_bytes("full: tail still sits at the very end", p + f.size() - 4, kTailBytes, 4);
    } else {
      fail("full: tail still sits at the very end", "frame smaller than one tail");
    }
  }

  // 58. a refused push must not touch the buffer
  {
    rm::FloatFrame f;
    f.reset();
    for (unsigned i = 0; i < rm::FloatFrame::kMaxFloats; ++i) {
      f.push(static_cast<float>(i));
    }
    uint8_t before[rm::FloatFrame::kCapacity] = {};
    const uint16_t sizeBefore = f.size();
    if (f.data() != nullptr) {
      std::memcpy(before, f.data(), f.size());
    }
    check_true("overflow push returns false", !f.push(123.0f), "overflow was accepted");
    check_int("overflow push leaves size() unchanged", f.size(), sizeBefore);
    check_true("overflow push leaves the bytes unchanged",
               f.data() != nullptr && std::memcmp(before, f.data(), f.size()) == 0,
               "buffer was modified by a refused push");
  }

  // 59. reset() after a full frame makes it reusable
  {
    rm::FloatFrame f;
    f.reset();
    for (unsigned i = 0; i < rm::FloatFrame::kMaxFloats; ++i) {
      f.push(1.0f);
    }
    f.reset();
    check_int("after reset: count() is 0", f.count(), 0);
    check_int("after reset: size() is 4", f.size(), 4);
    check_true("after reset: push works again", f.push(1.0f), "reset did not free the frame");
    check_int("after reset: count() is 1", f.count(), 1);
  }

  // 60. NaN / +Inf / -Inf are refused: they collide with the tail pattern
  {
    rm::FloatFrame f;
    f.reset();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    check_true("NaN is refused", !f.push(nan), "NaN was accepted");
    check_true("+Inf is refused", !f.push(inf), "+Inf was accepted");
    check_true("-Inf is refused", !f.push(-inf), "-Inf was accepted");
    check_int("refused values leave count() at 0", f.count(), 0);
    check_int("refused values leave size() at 4", f.size(), 4);
    check_bytes("refused values leave the frame untouched", f.data(), kTailBytes, 4);
  }

  // 61. +Inf really is the tail pattern - that is why the check above matters
  {
    const float inf = std::numeric_limits<float>::infinity();
    uint8_t bits[4] = {};
    std::memcpy(bits, &inf, 4);
    check_bytes("+Inf encodes as 00 00 80 7F (the tail)", bits, kTailBytes, 4);
  }

  // 62. after legal pushes the frame is still parseable: tail only at the end
  {
    rm::FloatFrame f;
    f.reset();
    f.push(1.5f);
    f.push(2.5f);
    check_int("2 floats: size() is 12", f.size(), 12);
    const uint8_t* p = f.data();
    bool tailOnlyAtEnd = (p != nullptr) && (f.size() == 12);
    if (p != nullptr) {
      for (uint16_t i = 0; i + 4 <= f.size(); i += 4) {
        const bool isTail =
            (p[i] == 0x00 && p[i + 1] == 0x00 && p[i + 2] == 0x80 && p[i + 3] == 0x7F);
        const bool isLast = (i + 4 == f.size());
        if (isTail != isLast) {
          tailOnlyAtEnd = false;
        }
      }
    }
    check_true("tail appears exactly once, at the end", tailOnlyAtEnd,
               "a tail pattern showed up in the middle");
  }

  std::printf("\npassed %d / %d\n", g_pass, g_pass + g_fail);
  return g_fail == 0 ? 0 : 1;
}
