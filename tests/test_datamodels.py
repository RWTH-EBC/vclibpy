import unittest
from random import random
import vclibpy
from vclibpy import HeatExchangerInputs


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


class TestHeatExchangerInputs(unittest.TestCase):
    def setUp(self):
        self.T_in = 273.15
        self.T_out = 278.15
        self.dT = 5.0
        self.m_flow = 0.5
        self.T_ambient = 300.0
        self.cp = 4184.0
        self.Q = 1000.0

    def test_init_with_T_in_dT(self):
        inputs = HeatExchangerInputs(T_in=self.T_in, dT=self.dT)
        self.assertEqual(inputs.T_in, self.T_in)
        self.assertEqual(inputs.T_out, self.T_in + self.dT)
        self.assertEqual(inputs.dT, self.dT)
        self.assertIsNone(inputs.m_flow)

    def test_init_with_T_out_dT(self):
        inputs = HeatExchangerInputs(T_out=self.T_out, dT=self.dT)
        self.assertEqual(inputs.T_out, self.T_out)
        self.assertEqual(inputs.T_in, self.T_out - self.dT)
        self.assertEqual(inputs.dT, self.dT)
        self.assertIsNone(inputs.m_flow)

    def test_init_with_T_in_T_out_dT(self):
        inputs = HeatExchangerInputs(T_in=self.T_in, T_out=self.T_out, dT=self.dT)
        self.assertEqual(inputs.T_in, self.T_in)
        self.assertEqual(inputs.T_out, self.T_out)
        self.assertEqual(inputs.dT, self.dT)
        self.assertIsNone(inputs.m_flow)

    def test_init_with_T_in_m_flow(self):
        inputs = HeatExchangerInputs(T_in=self.T_in, m_flow=self.m_flow)
        self.assertEqual(inputs.T_in, self.T_in)
        self.assertIsNone(inputs.T_out)
        self.assertIsNone(inputs.dT)
        self.assertEqual(inputs.m_flow, self.m_flow)

    def test_init_with_T_out_m_flow(self):
        inputs = HeatExchangerInputs(T_out=self.T_out, m_flow=self.m_flow)
        self.assertEqual(inputs.T_out, self.T_out)
        self.assertIsNone(inputs.T_in)
        self.assertIsNone(inputs.dT)
        self.assertEqual(inputs.m_flow, self.m_flow)

    def test_init_with_invalid_inputs(self):
        with self.assertRaises(ValueError):
            HeatExchangerInputs(dT=self.dT)
        with self.assertRaises(ValueError):
            HeatExchangerInputs(m_flow=self.m_flow)
        with self.assertRaises(ValueError):
            HeatExchangerInputs(T_in=self.T_in, T_out=self.T_out, m_flow=self.m_flow)

    def test_get_all_inputs(self):
        inputs = HeatExchangerInputs(T_in=self.T_in, T_out=self.T_out)
        T_in, T_out, dT, m_flow = inputs.get_all_inputs(cp=self.cp, Q=self.Q)
        self.assertEqual(T_in, self.T_in)
        self.assertEqual(T_out, self.T_out)
        self.assertEqual(dT, self.T_out - self.T_in)
        self.assertAlmostEqual(m_flow, self.Q / self.cp / dT, places=6)


if __name__ == "__main__":
    unittest.main()
