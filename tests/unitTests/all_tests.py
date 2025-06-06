import pytest

if __name__ == "__main__":
    # Explicitly specify the test files to run
    pytest.main(["tests/unitTests/pid_test.py"])
    pytest.main(["tests/unitTests/dc_motor_test.py"])
    pytest.main(["tests/unitTests/ziegler_nichols_test.py"])
    pytest.main(["tests/unitTests/brute_force_test.py"])
