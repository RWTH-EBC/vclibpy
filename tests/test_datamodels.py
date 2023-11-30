import unittest
from random import random
import vclibpy


class TestDataModels(unittest.TestCase):
    """Test-class for the datamodels module of vclibpy."""

    def setUp(self) -> None:
        self.fs_state = vclibpy.FlowsheetState()
        self.var = vclibpy.datamodels.Variable(name="test", value=random(), unit="K", description="dummy")

    def test_variable_set(self):
        self.fs_state.set(**self.var.__dict__)
        self.assertEqual(self.fs_state.test, self.var.value)
        self.assertEqual(self.fs_state.get_variables()["test"], self.var)
        self.assertEqual(self.fs_state.get_variable_names(), ["test"])
        self.assertEqual(self.fs_state._variables, {"test": self.var})
        # Update value
        self.fs_state.set(name="test", value=0)
        self.assertEqual(self.fs_state.test, 0)


if __name__ == "__main__":
    unittest.main()
