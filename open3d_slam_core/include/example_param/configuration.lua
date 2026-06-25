include "default/default_parameters.lua"


params = deepcopy(DEFAULT_PARAMETERS)-- 1. Збільшуємо буфери, щоб SLAM не втрачав кадри під час важких розрахунків мапера
params.odometry.odometry_buffer_size = 10
params.mapper_localizer.mapping_buffer_size = 10

-- 2. Змінюємо тип кропера на стандартний радіус (якщо MinMaxRadius не підтримується твоїм ядром)
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropper_type = "MinMaxRadius"
params.odometry.scan_processing.scan_cropping.cropper_type = "MinMaxRadius"

-- 3. Обрізаємо кімнату до реальних меж (4 метри — ідеально для ScanNet)
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 8.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 0.0

-- 4. Робимо адекватні ліміти по висоті стелі та підлоги (Z), замість 100 метрів
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.max_z = 3.5
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.min_z = -2.0

-- 5. Прискорюємо очищення простору від застарілих вокселів
params.map_builder.space_carving.carve_space_every_n_scans = 50
return params
