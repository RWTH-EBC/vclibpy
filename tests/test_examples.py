"""Module with functions to test the if examples are
executable."""
import unittest
import subprocess
import pathlib


class TestExamples(unittest.TestCase):
    """Test-class for the examples of vclibpy."""

    def setUp(self) -> None:
        self.timeout = 100  # Seconds which the script is allowed to run

    def _run_example(self, example, timeout=None):
        if timeout is None:
            timeout = self.timeout
        ex_py = pathlib.Path(__file__).absolute().parents[1].joinpath("examples", example)
        try:
            subprocess.check_output(['python', ex_py],
                                    stderr=subprocess.STDOUT,
                                    timeout=timeout)
        # except subprocess.TimeoutExpired:
        #     pass
        except subprocess.CalledProcessError as proc_err:
            raise Exception(proc_err.output.decode('utf-8')) from proc_err

    def test_e1_refrigerant_data(self):
        self._run_example(example="e1_refrigerant_data.py")

    def test_e2_compressor(self):
        self._run_example(example="e2_compressor.py")

    def test_e3_inputs_and_flowsheet_state(self):
        self._run_example(example="e3_inputs_and_flowsheet_state.py")

    def test_e4_heat_exchanger(self):
        self._run_example(example="e4_heat_exchanger.py")

    def test_e5_expansion_valve(self):
        self._run_example(example="e5_expansion_valve.py")

    def test_e6_simple_heat_pump(self):
        self._run_example(example="e6_simple_heat_pump.py")

    def test_e7_vapor_injection(self):
        self._run_example(example="e7_vapor_injection.py")


if __name__ == "__main__":
    unittest.main()
