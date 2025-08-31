#!/bin/bash
set -e

# 1️⃣ Создать пустой файл empty.cpp
FLANN_CPP_DIR="src/cpp"
EMPTY_FILE="$FLANN_CPP_DIR/empty.cpp"

mkdir -p "$FLANN_CPP_DIR"
touch "$EMPTY_FILE"
echo "// empty file to workaround build issues" > "$EMPTY_FILE"

echo "Created $EMPTY_FILE"

# 2️⃣ Изменить строки в CMakeLists.txt
CMAKE_FILE="$FLANN_CPP_DIR/CMakeLists.txt"

if [ ! -f "$CMAKE_FILE" ]; then
    echo "Error: $CMAKE_FILE not found!"
    exit 1
fi

# Функция для безопасной замены строки по содержимому
replace_line_content() {
    local file=$1
    local search=$2
    local replace=$3
    # sed inplace (-i) Linux вариант
    sed -i "s@${search}@${replace}@g" "$file"
}

# Заменяем все нужные строки
replace_line_content "$CMAKE_FILE" 'add_library(flann_cpp SHARED.*' 'add_library(flann_cpp SHARED "empty.cpp")'
replace_line_content "$CMAKE_FILE" 'add_library(flann_cpp SHARED empty.cpp.*' 'add_library(flann_cpp SHARED empty.cpp ${CPP_SOURCES})'
replace_line_content "$CMAKE_FILE" 'add_library(flann SHARED.*' 'add_library(flann SHARED "empty.cpp")'
replace_line_content "$CMAKE_FILE" 'add_library(flann SHARED empty.cpp.*' 'add_library(flann SHARED empty.cpp ${C_SOURCES})'

echo "Updated $CMAKE_FILE safely"
