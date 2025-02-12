from environment.MapLoader import MapLoader
from objectives.violation_number.oracles import RecordAnalyzer
from objectives.violation_number.oracles.impl.ComfortOracle import ComfortOracle
from objectives.violation_number.oracles.impl.SpeedingOracle import SpeedingOracle
from objectives.violation_number.oracles.impl.CollisionOracle import CollisionOracle
from objectives.violation_number.oracles.impl.JunctionLaneChangeOracle import JunctionLaneChangeOracle
from objectives.violation_number.oracles.impl.ModuleDelayOracle import ModuleDelayOracle
from objectives.violation_number.oracles.impl.ModuleOracle import ModuleOracle
from objectives.violation_number.oracles.impl.UnsafeLaneChangeOracle import UnsafeLaneChangeOracle

def measure_violations(map_name, scenario_record_path):
    MapLoader(map_name)

    target_oracles = [
        ComfortOracle(),
        CollisionOracle(),
        SpeedingOracle(),
        ModuleDelayOracle(),
        ModuleOracle(),
        UnsafeLaneChangeOracle(),
        JunctionLaneChangeOracle()
    ]

    analyzer = RecordAnalyzer(record_path=str(scenario_record_path), oracles=target_oracles)
    violations = analyzer.analyze()
    print(f"Violation Results: {[(violation.main_type, violation.key_label) for violation in violations]}")
    return violations



if __name__ == '__main__':
    map_name = "LEO-VM-00032" # This is a placeholder, the actual map name should be provided
    scenario_record_path = "data/records/sqlite.db3" # This is a placeholder, the actual path to the record should be provided
    measure_violations(map_name, scenario_record_path)