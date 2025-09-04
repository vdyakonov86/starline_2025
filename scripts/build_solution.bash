mkdir -p /starline/ws_solution/build/ && cd /starline/ws_solution/build/
cmake .. && make
./main /starline/maps/map_dense.pcd /starline/ws_solution/config.yaml