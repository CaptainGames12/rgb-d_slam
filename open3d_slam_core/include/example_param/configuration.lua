include "default/default_parameters.lua"


params = deepcopy(DEFAULT_PARAMETERS)
params.odometry.odometry_buffer_size = 1
params.mapper_localizer.mapping_buffer_size = 1
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropper_type = "MinMaxRadius"
params.odometry.scan_processing.scan_cropping.cropper_type = "MinMaxRadius"
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 100.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 0.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.max_z = 100.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.min_z = -100.0
params.map_builder.space_carving.carve_space_every_n_scans = 1000
return params
