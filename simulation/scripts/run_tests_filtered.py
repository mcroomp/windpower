#!/usr/bin/env python3
"""
run_tests_filtered.py — Pytest wrapper with clean agent-friendly output.

Runs pytest on the specified test path and filters output in real-time to show:
  - Test name and result (PASSED/FAILED/SKIPPED) as they complete
  - Failure details saved to temp file
  - Clean summary at end with temp file path for failures

IMPORTANT: Do NOT pipe this script's output! It already handles filtering,
formatting, and saving failures to temp files. Just run directly:

  python run_tests_filtered.py unit [pytest args...]
  python run_tests_filtered.py simtest [pytest args...]
  python run_tests_filtered.py "simulation/tests/simtests" -k test_foo [pytest args...]

Failures are automatically saved to: C:\\Temp\\pytest_YYYYMMDD_HHMMSS_XXXX.log
Filename is printed in the output for easy debugging.
"""

import subprocess
import sys
import re
import tempfile


def run_tests_filtered(test_type: str, pytest_args: list[str]) -> int:
    """Run pytest with live-streamed filtered output and save failures to temp file."""
    
    # Map test types to paths
    paths = {
        'unit': 'simulation/tests/unit',
        'simtest': 'simulation/tests/simtests',
    }
    
    test_path = paths.get(test_type, test_type)
    
    # Build pytest command
    cmd = [
        sys.executable, '-m', 'pytest',
        test_path,
        '-v',
        '--tb=short',
    ]
    
    # Add marker for unit tests
    if test_type == 'unit':
        cmd.extend(['-m', 'not simtest'])
    elif test_type == 'simtest':
        cmd.extend(['-m', 'simtest'])
    
    cmd.extend(pytest_args)
    
    print(f"[RUN] {' '.join(cmd)}\n", file=sys.stderr)
    
    # Run pytest with live output streaming
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    
    passed = failed = skipped = 0
    failed_tests = []
    temp_file = None
    temp_output = []
    
    # Stream and filter output line by line
    for line in proc.stdout:
        line = line.rstrip('\n')
        temp_output.append(line)
        
        # Match test results: test_name PASSED/FAILED/SKIPPED [duration or %]
        # Example: simulation\tests\unit\test_file.py::TestClass::test_name PASSED [  4%]
        match = re.search(r'::([\w:]+)\s+(PASSED|FAILED|SKIPPED)\s+\[', line)
        if match:
            test_name, status = match.groups()
            if status == 'PASSED':
                print(f"  [PASS] {test_name}")
                passed += 1
            elif status == 'FAILED':
                print(f"  [FAIL] {test_name}")
                failed_tests.append(test_name)
                failed += 1
            elif status == 'SKIPPED':
                print(f"  [SKIP] {test_name}")
                skipped += 1
    
    exit_code = proc.wait()
    
    # Save failures to temp file if any
    if failed_tests:
        temp_file = tempfile.NamedTemporaryFile(
            mode='w',
            suffix='.txt',
            prefix='pytest_failures_',
            delete=False,
            encoding='utf-8'
        )
        
        # Write failed test names
        temp_file.write(f"FAILED TESTS ({len(failed_tests)}):\n")
        temp_file.write("=" * 76 + "\n")
        for test_name in failed_tests:
            temp_file.write(f"  - {test_name}\n")
        
        # Write full pytest output
        temp_file.write("\n\nFULL PYTEST OUTPUT:\n")
        temp_file.write("=" * 76 + "\n")
        temp_file.write('\n'.join(temp_output))
        
        temp_file.close()
    
    # Print summary
    print("\n" + "=" * 76)
    parts = []
    if passed:
        parts.append(f"{passed} passed")
    if failed:
        parts.append(f"{failed} failed")
    if skipped:
        parts.append(f"{skipped} skipped")
    
    summary = ", ".join(parts) if parts else "no tests collected"
    print(f"SUMMARY: {summary}")
    
    if temp_file:
        print(f"FAILURES SAVED TO: {temp_file.name}")
    
    print("=" * 76)
    
    return exit_code


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} unit|simtest|<path> [pytest args...]")
        print()
        print("Examples:")
        print(f"  {sys.argv[0]} unit                          # Run all unit tests")
        print(f"  {sys.argv[0]} simtest                       # Run all simtests")
        print(f"  {sys.argv[0]} simtest -k test_steady_flight # Run specific simtest")
        print(f"  {sys.argv[0]} simtest -x                    # Stop on first failure")
        sys.exit(1)
    
    test_type = sys.argv[1]
    pytest_args = sys.argv[2:]
    
    exit_code = run_tests_filtered(test_type, pytest_args)
    sys.exit(exit_code)
