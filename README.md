# Инструкция по запуску
1. Сборка образа: docker build -t starline .
2. Запуск контейнера: bash scripts/run.bash starline starline
3. Сборка пакетов:
  - . scripts/build_livox.bash
  - . scripts/build_fastlio.bash
  - . scripts/build_solution.bash
4. Настройка окружения: source setup.bash
5. Запуск решения:
  - Запуск fast-lio для построения карты: . scripts/start_fastlio.bash
  - Проигрывание bag-файла: . scripts/rosbag_play.bash
  - После окончания проигрывания сохранить карту: . scripts/save_3dmap.bash
  - Запустить решение для кластеризации крестиков и нахождения их геометрического центра: . scripts/start_solution.bash