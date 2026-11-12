/* test_knob_names.c -- host test for s32_wasm_knob_names (../src/s32_wasm.c), run by the Makefile's test-host
 * target. The function under test is compiled with s32board_knob_names renamed to test_knob_names, defined here:
 * a name list far longer than the bench's real one, whose JSON is larger than 4096 bytes. The list must come back
 * complete (closing ']' included), in order, built once. */
#include <stdio.h>
#include <string.h>

const char *s32_wasm_knob_names(void);

#define TEST_KNOBS 300                          /* ~31 chars a name -> ~9 KB of JSON, past the old 4 KB buffer */
static char NAME[TEST_KNOBS][32];
static const char *NAMES[TEST_KNOBS + 1];

const char *const *test_knob_names(void) {
  for (int i = 0; i < TEST_KNOBS; i++) {
    snprintf(NAME[i], sizeof NAME[i], "S32_TEST_KNOB_%04d_LONGTAIL", i);
    NAMES[i] = NAME[i];
  }
  NAMES[TEST_KNOBS] = NULL;
  return NAMES;
}

#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #cond); return 1; } } while (0)

int main(void) {
  test_knob_names();                            /* populate NAME before sizing the expected JSON */
  size_t want = 1;                              /* '[' */
  for (int i = 0; i < TEST_KNOBS; i++) want += strlen(NAME[i]) + 2 + (i ? 1 : 0);   /* "name" and a ',' */
  want += 1;                                    /* ']' */

  const char *j = s32_wasm_knob_names();
  CHECK(j != NULL);
  CHECK(strlen(j) == want);
  CHECK(j[0] == '[' && j[want - 1] == ']' && j[want] == '\0');
  const char *p = j + 1;                        /* every name present, quoted, in the provider's order */
  for (int i = 0; i < TEST_KNOBS; i++) {
    char frag[40];
    snprintf(frag, sizeof frag, "%s\"%s\"", i ? "," : "", NAME[i]);
    CHECK(strncmp(p, frag, strlen(frag)) == 0);
    p += strlen(frag);
  }
  CHECK(strcmp(p, "]") == 0);
  CHECK(s32_wasm_knob_names() == j);            /* built once: the same pointer answered again */
  printf("ok: %d knob names, %zu JSON bytes, complete and built once\n", TEST_KNOBS, want);
  return 0;
}
