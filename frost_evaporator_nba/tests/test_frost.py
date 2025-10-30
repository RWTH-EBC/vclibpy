import pytest
import numpy as np
from unittest.mock import MagicMock
import os
import sys

# Get the absolute path of the project's root directory (one level up)
project_root = os.path.abspath('..')
# Add the project root to the Python path if it's not already there
if project_root not in sys.path:
    sys.path.append(project_root)

from frost_evaporator import *
from frost_evaporator.datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,)
from frost_evaporator.frost import FrostModel




@pytest.fixture
def mock_params():
    """Creates a mock parameter object. A consistent fake set of parameters for the FrostModel"""
    params = MagicMock(spec=FrostEvaporatorParameters)
    
    # Set up the default correlation choices for our tests
    params.frost_density_correlation_choice = "jonas_diss"
    params.frost_thickness_correlation_choice = "jonas_diss"
    params.frost_conductivity_correlation_choice = "A"
    return params

@pytest.fixture
def frost_model(mock_params):
    """
    Creates a FrostModel instance using the 'mock_params' fixture 
    and a fixed delta_t.
    Gives every test a fresh and identical instance of FrostModel.
    """
    # This function depends on the 'mock_params' fixture above
    return FrostModel(parameters=mock_params, delta_t=1.0)


def test_calculate_orchestration(frost_model, mocker):
    """
    Tests that the main 'calculate' method calls helpers and updates state in the correct order.
    """
    # 1. ARRANGE
    
    # Mock the state object
    mock_state = MagicMock(spec=FrostEvaporatorState())

    # Define the "previous" values that the method will read
    mock_state.hmt.T_frost_surface = 270.0   # Previous surface temp
    mock_state.frost.thickness = 0.001       # Previous thickness
    mock_state.hmt.m_dot_frost_flux = 0.0001 # Frost flux
    
    # Patch the helper methods (replace real functions with simple return to check orchestration - not calculations)
    m_density = mocker.patch.object(
        frost_model, 'calculate_density', return_value=500.0
    )
    m_thickness = mocker.patch.object(
        frost_model, 'calculate_thickness', return_value=0.002
    )
    m_k_frost = mocker.patch.object(
        frost_model, 'calculate_k_frost', return_value=0.5
    )

    # 2. ACT
    frost_model.calculate(state=mock_state)

    # 3. ASSERT
    
    # Assert helpers were called with correct values
    m_density.assert_called_once_with(T_frost_surface=270.0)
    m_thickness.assert_called_once_with(
        prev_thickness=0.001,
        m_dot_frost_flux=0.0001,
        new_density=500.0
    )
    m_k_frost.assert_called_once_with(new_density=500.0)
    
    # --- Assert state was updated with new values ---
    # We check that the .set() method on the sub-mock was called.
    mock_state.frost.set.assert_any_call("density", 500.0)
    mock_state.frost.set.assert_any_call("thickness", 0.002)
    mock_state.frost.set.assert_any_call("k_frost", 0.5)
    
    # Ensure only these 3 calls were made
    assert mock_state.frost.set.call_count == 3


def test_calculate_density_jonas_diss(frost_model):
    """
    Tests the 'jonas_diss' density calculation with a known value (0°C).
    """
    # Arrange
    T_surface_celsius_0 = 273.15  # 0°C
    
    # Act
    # 650 * exp(0.277 * (273.15 - 273.15)) = 650
    result = frost_model.calculate_density(T_frost_surface=T_surface_celsius_0)
    
    # Assert
    assert np.isclose(result, 650.0)

def test_calculate_density_invalid_choice(mock_params):
    """
    Tests that an unknown correlation choice raises a ValueError.
    """
    # Arrange
    mock_params.frost_density_correlation_choice = "unknown_choice"
    model = FrostModel(parameters=mock_params, delta_t=1.0)
    
    # Act & Assert
    with pytest.raises(ValueError, match="Unknown frost density correlation"):
        model.calculate_density(T_frost_surface=270.0)

def test_calculate_thickness_jonas_diss(frost_model):
    """
    Tests the 'jonas_diss' thickness calculation (Euler step).
    """
    # Arrange
    # Use delta_t = 1.0 (from fixture)
    prev_thickness = 0.001     # 1 mm
    m_dot_frost_flux = 0.0001  # kg/(m^2*s)
    new_density = 500.0        # kg/m^3
    
    # Act
    # new = 0.001 + (0.0001 * 1.0) / 500.0 = 0.0010002
    result = frost_model.calculate_thickness(
        prev_thickness=prev_thickness,
        m_dot_frost_flux=m_dot_frost_flux,
        new_density=new_density
    )
    
    # Assert
    assert np.isclose(result, 0.0010002)

def test_calculate_thickness_invalid_choice(mock_params):
    """
    Tests that an unknown correlation choice raises a ValueError.
    """
    # Arrange
    mock_params.frost_thickness_correlation_choice = "unknown_choice"
    model = FrostModel(parameters=mock_params, delta_t=1.0)
    
    # Act & Assert
    with pytest.raises(ValueError, match="Unknown frost thickness correlation"):
        model.calculate_thickness(prev_thickness = 1, m_dot_frost_flux = 1, new_density = 1)

def test_calculate_k_frost_correlation_A(frost_model):
    """
    Tests the 'A' conductivity calculation.
    """
    # Arrange
    new_density = 650.0
    
    # Act
    # 1.202e-3 * (650 ** 0.963)
    expected_result = 1.202e-3 * (650.0 ** 0.963) # approx 0.6585
    result = frost_model.calculate_k_frost(new_density=new_density)
    
    # Assert
    assert np.isclose(result, expected_result)

def test_calculate_k_frost_invalid_choice(mock_params):
    """
    Tests that an unknown correlation choice raises a ValueError.
    """
    # Arrange
    mock_params.frost_conductivity_correlation_choice = "unknown_choice"
    model = FrostModel(parameters=mock_params, delta_t=1.0)
    
    # Act & Assert
    with pytest.raises(ValueError, match="Unknown frost conductivity correlation"):
        model.calculate_k_frost(new_density = 1)