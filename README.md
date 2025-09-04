# Инструкция по запуску
1. Сборка образа: docker build -t starline .
2. Запуск контейнера: bash scripts/run.bash starline starline
3. Сборка пакетов:
  - . scripts/build_livox.bash
  - . scripts/build_fastlio.bash
  - . scripts/build_solution.bash
4. Настройка окружения: source scripts/setup.bash
5. Запуск решения:
  - Запуск fast-lio для построения карты: ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
  - Проигрывание bag-файла: ros2 bag play bag_files/tb_office_v02/
  - После окончания проигрывания сохранить карту: ros2 service call /map_save std_srvs/srv/Trigger {}
  - Запустить решение для кластеризации крестиков и нахождения их геометрического центра: . scripts/start_solution.bash