#!/bin/bash

# 设置Git子模块的脚本
# 这个脚本将为每个算法仓库设置Git子模块

echo "开始设置Git子模块..."

# 定义子模块映射
declare -A submodules=(
    ["DM_VIO"]="https://github.com/lukasvst/dm-vio.git"
    ["OpenVINS"]="https://github.com/rpng/open_vins.git"
    ["VINS-Fusion"]="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git"
    ["airslam"]="https://github.com/zdzhaoyong/AirSLAM.git"
    ["basalt"]="https://gitlab.com/VladyslavUsenko/basalt.git"
    ["dso"]="https://github.com/JakobEngel/dso.git"
    ["kimera"]="https://github.com/MIT-SPARK/Kimera-VIO.git"
    ["msckf"]="https://github.com/KumarRobotics/msckf_vio.git"
    ["orb3"]="https://github.com/UZ-SLAMLab/ORB_SLAM3.git"
    ["orbslam3"]="https://github.com/UZ-SLAMLab/ORB_SLAM3.git"
    ["plslam"]="https://github.com/rubengooj/PL-SLAM.git"
    ["schur"]="https://github.com/uzh-rpg/SchurVINS.git"
    ["svo2"]="https://github.com/uzh-rpg/rpg_svo_pro_open.git"
    ["vins"]="https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git"
)

# 为每个子模块执行操作
for dir in "${!submodules[@]}"; do
    url="${submodules[$dir]}"
    
    if [ -d "$dir" ]; then
        echo "处理 $dir..."
        
        # 进入目录并检查Git状态
        cd "$dir"
        
        # 检查是否有未提交的更改
        if ! git diff --quiet || ! git diff --cached --quiet; then
            echo "  $dir 有未提交的更改，正在提交..."
            git add .
            git commit -m "Add Docker configuration and calibration files

- Add docker-compose.yml and Dockerfile
- Add dataset calibration configurations  
- Add setup and run scripts
- Add results and build configurations

This commit includes all modifications made for algorithm testing and optimization."
        fi
        
        # 返回上级目录
        cd ..
        
        # 添加为子模块
        echo "  添加 $dir 作为子模块..."
        git submodule add "$url" "$dir"
        
    else
        echo "目录 $dir 不存在，跳过..."
    fi
done

echo "子模块设置完成！"
echo ""
echo "下一步："
echo "1. 检查子模块状态: git submodule status"
echo "2. 提交子模块配置: git add .gitmodules && git commit -m 'Add submodules'"
echo "3. 推送到GitHub: git push origin master"
