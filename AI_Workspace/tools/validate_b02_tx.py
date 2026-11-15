"""B02 acceptance: real VOFA/USB TX bodies, ARMCC + Unicorn, no hardware.

The UART/USB drivers defer reading the submitted pointer until completion. VOFA
storage has canaries around its real 256-byte payload; only that declaration is
instrumented. The USB harness also checks the content of an in-flight transfer
after a second write is rejected, not merely its pointer/length.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

from audit import read_source
from project_paths import ROOT, project_path
from validate_invariants import functions
from validate_rx_chain import build, usb_source


def vofa_source():
    path = ROOT / "User/Bsp/bsp_vofa+.c"
    body = functions(read_source(path))["Vofa_JustFloat"]
    declaration = "static uint8_t buf[256];"
    if body.count(declaration) != 1:
        raise RuntimeError("VOFA payload declaration changed: review instrumentation")
    body = body.replace(declaration, "")
    import re
    body = re.sub(r"\bbuf\b", "guard.payload", body)
    return r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define HAL_UART_STATE_READY 0u
#define HAL_UART_STATE_BUSY_TX 1u
#define HAL_OK 0u
#define HAL_ERROR 1u
typedef struct { uint32_t gState; } UART_HandleTypeDef;
UART_HandleTypeDef huart10;
static volatile uint32_t vofa_dropped, vofa_sent;
/* The large trailing canary safely exposes the complete old uint8_t range. */
static struct { uint8_t before[16], payload[256], after[1024]; } guard;
static float input[255];
static uint8_t *pending;
static uint16_t pending_length;
static uint32_t sends, fail_send;
uint32_t HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *buf, uint16_t length) {
  (void)uart;
  ++sends;
  if (fail_send) return HAL_ERROR;
  pending = buf;
  pending_length = length;
  huart10.gState = HAL_UART_STATE_BUSY_TX;
  return HAL_OK;
}
volatile struct { uint32_t status, checks, failure; } test_result;
''' + "void Vofa_JustFloat(float *_data, uint8_t _num) " + body + r'''
static void check(int ok, uint32_t id) {
  ++test_result.checks;
  if (!ok && !test_result.failure) test_result.failure = id;
}
static int all_bytes(const uint8_t *p, uint32_t size, uint8_t value) {
  uint32_t i;
  for (i = 0; i < size; ++i) if (p[i] != value) return 0;
  return 1;
}
static void reset(void) {
  memset(&guard, 0xA5, sizeof(guard));
  huart10.gState = HAL_UART_STATE_READY;
  sends = fail_send = vofa_dropped = vofa_sent = 0;
  pending = 0; pending_length = 0;
}
void test_entry(void) {
  uint32_t n, i;
  static uint8_t saved[256];
  const uint8_t tail[4] = {0, 0, 0x80, 0x7F};
  for (i = 0; i < 255; ++i) input[i] = (float)i + 0.25f;
  reset(); Vofa_JustFloat(0, 10);
  check(!sends && vofa_dropped == 1, 1);
  reset(); Vofa_JustFloat(input, 0);
  check(!sends && vofa_dropped == 1, 2);
  for (n = 1; n <= 255; ++n) {
    reset();
    Vofa_JustFloat(input, (uint8_t)n);
    check(all_bytes(guard.before, sizeof(guard.before), 0xA5), 1000+n);
    check(all_bytes(guard.after, sizeof(guard.after), 0xA5), 2000+n);
    if (n <= 63) {
      check(sends == 1 && vofa_sent == 1 && !vofa_dropped, 3000+n);
      check(pending_length == 4*n+4, 4000+n);
      check(memcmp(pending, input, 4*n) == 0, 5000+n);
      check(memcmp(pending+4*n, tail, 4) == 0, 6000+n);
    } else {
      check(!sends && !vofa_sent && vofa_dropped == 1, 7000+n);
      check(all_bytes(guard.payload, sizeof(guard.payload), 0xA5), 8000+n);
    }
  }
  reset(); Vofa_JustFloat(input, 10);
  memcpy(saved, pending, pending_length);
  Vofa_JustFloat(input+10, 10);
  check(sends == 1 && vofa_dropped == 1 && vofa_sent == 1, 9001);
  check(memcmp(saved, pending, pending_length) == 0, 9002);
  huart10.gState = HAL_UART_STATE_READY;
  Vofa_JustFloat(input+10, 10);
  check(sends == 2 && memcmp(pending, input+10, 40) == 0, 9003);
  reset(); fail_send = 1; Vofa_JustFloat(input, 10);
  check(sends == 1 && !vofa_sent && vofa_dropped == 1, 9004);
  test_result.status = 0x600D;
  __asm("bkpt 0");
}
'''


def usb_busy_source():
    source = usb_source()
    marker = "test_result.status = 0x600D;"
    if source.count(marker) != 1:
        raise RuntimeError("USB harness changed: review in-flight content probe")
    source = source.replace(marker, r'''
  reset_usb();
  check(usb_cdc_port_write(NULL, NULL, 4u) == 0u && usb_stats.tx_rejected == 1u, 20);
  reset_usb();
  check(usb_cdc_port_write(NULL, stack_buf, 0u) == 0u && usb_stats.tx_rejected == 1u, 21);
  reset_usb();
  {
    static uint8_t exact[512];
    memset(exact, 0x3C, sizeof(exact));
    check(usb_cdc_port_write(NULL, exact, sizeof(exact)) == 512u &&
          usb_pending_len == 512u && usb_stats.tx_frames == 1u, 22);
    memset(exact, 0x99, sizeof(exact));
    check(usb_pending_ptr[0] == 0x3Cu && usb_pending_ptr[511] == 0x3Cu, 23);
  }
''' + marker)
    return source


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    args = parser.parse_args()
    cache, results = project_path(args.cache), project_path(args.results)
    outcomes = []
    for name, source in (("b02-vofa-boundary", vofa_source()),
                         ("b02-usb-busy-content", usb_busy_source())):
        result = build(name, source, cache, results)
        for ext in (".c", ".axf"):
            shutil.copy2(cache / (name + ext), results / (name + ext))
        outcomes.append(result)
    evidence = {"scope": "real firmware function bodies; HAL/USB stand-ins; no hardware",
                "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in (ROOT / "User/Bsp/bsp_vofa+.c",
                                            ROOT / "User/Communication/example/device/usb_cdc_port.c")},
                "results": outcomes}
    (results / "tx-summary.json").write_text(json.dumps(evidence, indent=2), encoding="utf-8")
    print(json.dumps(evidence, indent=2))
    return 0 if all(r["passed"] for r in outcomes) else 1


if __name__ == "__main__":
    raise SystemExit(main())
