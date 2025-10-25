#!/bin/bash

echo "[INFO] Replacing deprecated OpenCV macros..."

FILES=$(grep -rl "CV_" ./VINS-Fusion)

REPLACE_MAP=(
    "CV_AA=cv::LINE_AA"
    "CV_BGR2GRAY=cv::COLOR_BGR2GRAY"
    "CV_GRAY2BGR=cv::COLOR_GRAY2BGR"
    "CV_GRAY2RGB=cv::COLOR_GRAY2RGB"
    "CV_RGB2GRAY=cv::COLOR_RGB2GRAY"
    "CV_FONT_HERSHEY_SIMPLEX=cv::FONT_HERSHEY_SIMPLEX"
    "CV_TERMCRIT_EPS=cv::TermCriteria::EPS"
    "CV_TERMCRIT_ITER=cv::TermCriteria::MAX_ITER"
    "CV_THRESH_BINARY=cv::THRESH_BINARY"
    "CV_THRESH_BINARY_INV=cv::THRESH_BINARY_INV"
    "CV_ADAPTIVE_THRESH_MEAN_C=cv::ADAPTIVE_THRESH_MEAN_C"
    "CV_RETR_CCOMP=cv::RETR_CCOMP"
    "CV_CHAIN_APPROX_SIMPLE=cv::CHAIN_APPROX_SIMPLE"
    "CV_SHAPE_RECT=cv::MORPH_RECT"
    "CV_SHAPE_CROSS=cv::MORPH_CROSS"
    "CV_CALIB_CB_ADAPTIVE_THRESH=cv::CALIB_CB_ADAPTIVE_THRESH"
    "CV_CALIB_CB_NORMALIZE_IMAGE=cv::CALIB_CB_NORMALIZE_IMAGE"
    "CV_CALIB_CB_FILTER_QUADS=cv::CALIB_CB_FILTER_QUADS"
    "CV_CALIB_CB_FAST_CHECK=cv::CALIB_CB_FAST_CHECK"
)

for file in $FILES; do
    for map in "${REPLACE_MAP[@]}"; do
        OLD=${map%%=*}
        NEW=${map#*=}
        sed -i "s/\b$OLD\b/$NEW/g" "$file"
    done
    echo "[OK] Processed $file"
done

echo "[DONE] All CV_ macros replaced with OpenCV 4 syntax."

