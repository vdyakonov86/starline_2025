# Инструкция по запуску
1. Сборка образа: docker build -t starline .
2. Запуск контейнера: 
  cd /starline/ && bash scripts/run.bash starline starline
3. Клонирование необходимых пакетов (FAST-LIO, Livox-SDK2, livox_ros_driver2): 
  cd /starline/ && . scripts/download_packages.bash
4. Сборка пакетов:
  - cd /starline/ && . scripts/build_livox.bash
  - cd /starline/ && . scripts/build_fastlio.bash
  - cd /starline/ && . scripts/build_solution.bash
5. Настройка окружения:
    cd /starline/ && source scripts/setup.bash
6. Запуск решения:
  - Запуск fast-lio для построения карты: 
      ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
  - Проигрывание bag-файла:
      ros2 bag play bag_files/tb_office_v02/
  - После окончания проигрывания сохранить карту: 
      ros2 service call /map_save std_srvs/srv/Trigger {}
  - Запустить решение для нахождения крестиков и их геометрического центра: 
      cd /starline/ && . scripts/start_solution.bash