import ansys.fluent.core as pyfluent

session = pyfluent.launch_fluent(precision="double", processor_count=8)
session.file.read_case_data("res_drone_8040_7445_pt.cas.h5")