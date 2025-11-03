import numpy as np
from typing import Protocol, Any, Dict, Tuple, Optional
from geometry import defineDrone
from solver_quadcopter import QuadSolver
from solver_drone import DroneSolver
from solver_helicopter import HelicopterSolver
import json



class DroneStrategy:
    def build_aircraft(self, config, main_RPM, small_RPM: dict | str) -> Any:
        return defineDrone(config, main_RPM=main_RPM, small_RPM=small_RPM)

    def build_solver(self, aircraft: Any, output_dir: str = ".") -> Any:
        if output_dir is None:
            raise ValueError("output_dir must be specified for DroneSolver.")
        return DroneSolver(aircraft, output_dir=output_dir)

    def apply_rpms(self, aircraft: Any, rpm_main: float, rpm_aux: float = 0.0) -> None:
        aircraft.main_prop.RPM = rpm_main
        for sp in getattr(aircraft, "small_props", []):
            sp.RPM = rpm_aux

    def label(self) -> str:
        return "drone"
    
class HelicopterStrategy:
    """Glue code for your 'main + tail rotor' helicopter."""
    def build_aircraft(self, config, main_RPM: Dict | str) -> Any:
        return defineDrone(config, main_RPM=main_RPM)

    def build_solver(self, aircraft: Any, output_dir: str = ".") -> Any:
        return HelicopterSolver(aircraft, output_dir=output_dir)

    def label(self) -> str:
        return "helicopter"
    
class QuadcopterStrategy:
    def build_aircraft(self, config, main_RPM: Dict | str) -> Any:
        return defineDrone(config, main_RPM=main_RPM)

    def build_solver(self, aircraft: Any, output_dir: str = ".") -> Any:
        return QuadSolver(aircraft, output_dir=output_dir)

    def label(self) -> str:
        return "quadcopter"