#!/bin/bash

BASE_DIR="/home/levy/data"
TUM_BASE_DIR="/home/levy/pcl_ws/src/pcl_01/data"

# 遍历日期目录
for DATE_DIR in $(ls -d ${BASE_DIR}/*/); do
    DATE=$(basename $DATE_DIR)
    echo "Checking date: $DATE"

    # 遍历数据文件
    for BAG_FILE in ${DATE_DIR}*.bag; do
        if [ -f "$BAG_FILE" ]; then
            DATA=$(basename $BAG_FILE | cut -d '_' -f 1)
            TUM_FILE="${TUM_BASE_DIR}/${DATE}/${DATA}_gt.txt"

            if [ -f "$TUM_FILE" ]; then
                echo "Found valid data: Date=${DATE}, Data=${DATA}"
            fi
        fi
    done
done

