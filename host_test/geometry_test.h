#pragma once

/**
 * @brief Run all geometry unit tests.
 *
 * Prints PASS/FAIL for each test to Serial in the format:
 *   [GTEST] Test N (name): PASS
 *   [GTEST] Test N (name): FAIL
 *   [GTEST] ===== M/5 tests passed =====
 *
 * Sub-assertion failures print diagnostic lines:
 *   [GTEST] FAIL Test N: description of failure
 *
 * Call from setup() when TEST_MODE is defined.
 *
 * @return Number of failed tests (0 = all passed).
 */
int geometry_test_runAll();
