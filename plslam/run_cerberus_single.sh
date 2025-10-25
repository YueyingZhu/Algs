#!/bin/bash

# PL-SLAM CERBERUS 单序列运行脚本
# 使用方法: ./run_cerberus_single.sh [seq] [mode] [gui]

SEQ=${1:-seq7}
MODE=${2:-stereo}
GUI=${3:-false}

echo "=========================================="
echo "PL-SLAM CERBERUS 运行脚本"
echo "序列: $SEQ"
echo "模式: $MODE"
echo "GUI: $GUI"
echo "=========================================="

# 设置数据集路径
DATASET_PATH="/datasets/CERBERUS/$SEQ"
RESULTS_DIR="/ws/results/CERBERUS/$MODE/$SEQ"

# 创建结果目录
mkdir -p $RESULTS_DIR

# 设置配置文件和词汇表路径
CONFIG_FILE="/ws/pl-slam/config/config/config_cerberus.yaml"
VOCAB_P="/ws/pl-slam/vocabulary/mapir_orb.yml"
VOCAB_L="/ws/pl-slam/vocabulary/mapir_lsd.yml"

echo "数据集路径: $DATASET_PATH"
echo "结果目录: $RESULTS_DIR"
echo "配置文件: $CONFIG_FILE"

# 检查文件是否存在
if [ ! -d "$DATASET_PATH" ]; then
    echo "错误: 数据集路径不存在: $DATASET_PATH"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "错误: 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

if [ ! -f "$DATASET_PATH/dataset_params.yaml" ]; then
    echo "错误: 数据集参数文件不存在: $DATASET_PATH/dataset_params.yaml"
    exit 1
fi

# 设置库路径
export LD_LIBRARY_PATH="/ws/pl-slam/lib:/ws/pl-slam/3rdparty/DBoW2/lib:/ws/pl-slam/3rdparty/line_descriptor/lib:/ws/stvo-pl/lib:$LD_LIBRARY_PATH"

# 切换到工作目录
cd /ws/pl-slam/build

echo "开始运行PL-SLAM..."
echo "=========================================="

# 运行PL-SLAM
if [ "$GUI" = "true" ]; then
    echo "启用GUI模式..."
    ./plslam_dataset $DATASET_PATH -c $CONFIG_FILE -o 0 -s 1
else
    echo "无GUI模式..."
    ./plslam_dataset $DATASET_PATH -c $CONFIG_FILE -o 0 -s 1 > $RESULTS_DIR/plslam.log 2>&1
fi

# 检查运行结果
if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "PL-SLAM 运行完成!"
    
    # 查找轨迹文件
    TRAJECTORY_FILE=""
    if [ -f "trajectory.txt" ]; then
        TRAJECTORY_FILE="trajectory.txt"
    elif [ -f "../trajectory.txt" ]; then
        TRAJECTORY_FILE="../trajectory.txt"
    else
        echo "警告: 未找到轨迹文件"
        exit 1
    fi
    
    # 移动轨迹文件到结果目录
    cp $TRAJECTORY_FILE $RESULTS_DIR/trajectory.txt
    echo "轨迹文件已保存到: $RESULTS_DIR/trajectory.txt"
    
    # 显示轨迹信息
    echo "=========================================="
    echo "轨迹文件信息:"
    wc -l $RESULTS_DIR/trajectory.txt
    head -5 $RESULTS_DIR/trajectory.txt
    
    # 进行evo评估
    echo "=========================================="
    echo "开始evo评估..."
    
    # 检查ground truth文件（使用svo2的gt_tum.txt作为参考）
    GT_TUM="/home/zyy/Algs/svo2/Results_svo2/CERBERUS/stereo+imu/${SEQ}/gt_tum.txt"
    GT_CSV="/datasets/CERBERUS/${SEQ}_gt.csv"
    
    # 如果svo2的gt_tum.txt不存在，则使用原始CSV转换
    if [ ! -f "$GT_TUM" ]; then
        echo "svo2 gt_tum.txt不存在，尝试使用原始CSV转换..."
        GT_TUM="$RESULTS_DIR/gt_tum.txt"
        
        if [ -f "$GT_CSV" ]; then
            echo "转换ground truth格式..."
            python3 /ws/convert_gt.py "$GT_CSV" "$GT_TUM"
        fi
    fi
    
    # 准备宿主机评估
    echo "准备宿主机evo评估..."
    HOST_RESULTS_DIR="$HOME/Algs/plslam/Results_plslam/CERBERUS/$MODE/$SEQ"
    HOST_TRAJECTORY="$HOST_RESULTS_DIR/trajectory.txt"
    HOST_GT_TUM="$HOST_RESULTS_DIR/gt_tum.txt"
    
    # 确保宿主机结果目录存在
    mkdir -p "$HOST_RESULTS_DIR"
    
    # 复制轨迹文件到宿主机
    cp "$RESULTS_DIR/trajectory.txt" "$HOST_TRAJECTORY"
    
    # 复制或转换GT文件到宿主机
    if [ -f "$GT_TUM" ]; then
        cp "$GT_TUM" "$HOST_GT_TUM"
    else
        echo "转换ground truth格式到宿主机..."
        python3 /ws/convert_gt.py "$GT_CSV" "$HOST_GT_TUM"
    fi
    
    if [ -f "$HOST_GT_TUM" ] && [ -f "$HOST_TRAJECTORY" ]; then
        echo "找到ground truth文件: $HOST_GT_TUM"
        
        # 创建宿主机评估脚本
        EVO_SCRIPT="$HOST_RESULTS_DIR/run_evo.sh"
        cat > "$EVO_SCRIPT" << 'EOF'
#!/bin/bash
cd "$(dirname "$0")"

# 创建临时文件存储evo输出
EVO_OUTPUT="./evo_output.txt"
EVO_ERROR="./evo_error.txt"

# 运行evo_ape并捕获输出
evo_ape tum gt_tum.txt trajectory.txt --align --t_max_diff 0.2 2>$EVO_ERROR >$EVO_OUTPUT

if [ $? -eq 0 ]; then
    echo "evo评估成功完成"
    
    # 提取关键指标（evo输出是tab分隔的）
    RMSE=$(grep "rmse" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
    MEAN=$(grep "mean" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
    MAX=$(grep "max" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
    MIN=$(grep "min" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
    
    echo "评估结果:"
    echo "  RMSE: $RMSE m"
    echo "  Mean: $MEAN m"
    echo "  Max:  $MAX m"
    echo "  Min:  $MIN m"
    
    # 判断结果是否合理
    RMSE_NUM=$(echo $RMSE | sed 's/[^0-9.]//g')
    REASONABLE_THRESHOLD=1.0
    
    echo "结果合理性检查..."
    
    if (( $(echo "$RMSE_NUM < $REASONABLE_THRESHOLD" | bc -l) )); then
        echo "✅ 结果合理 (RMSE: $RMSE < $REASONABLE_THRESHOLD)"
        
        # 保存APE结果
        cp $EVO_OUTPUT ./ape_results.txt
        
        # 尝试保存绘图（忽略错误）
        evo_ape tum gt_tum.txt trajectory.txt --align --t_max_diff 0.2 \
            --plot --plot_mode xyz \
            --save_results ./ape_results.zip \
            --save_plot ./ape_plot.pdf 2>/dev/null || true
        
        # 尝试轨迹对比图
        evo_traj tum trajectory.txt gt_tum.txt \
            --ref gt_tum.txt \
            --align --t_max_diff 0.2 \
            --plot --plot_mode xyz \
            --save_plot ./traj_plot.pdf 2>/dev/null || true
        
        echo "✅ evo结果已保存"
    else
        echo "❌ 结果不合理 (RMSE: $RMSE >= $REASONABLE_THRESHOLD)"
    fi
else
    echo "❌ evo评估失败"
    cat $EVO_ERROR
fi
EOF
        
        chmod +x "$EVO_SCRIPT"
        
        # 退出容器并在宿主机上运行evo评估
        echo "退出容器，在宿主机上运行evo评估..."
        echo "请在宿主机上运行: $EVO_SCRIPT"
        echo "或者运行: cd $HOST_RESULTS_DIR && ./run_evo.sh"
        
        # 创建容器内的评估总结
        echo "PL-SLAM CERBERUS 评估结果" > $RESULTS_DIR/evaluation_summary.txt
        echo "序列: $SEQ" >> $RESULTS_DIR/evaluation_summary.txt
        echo "模式: $MODE" >> $RESULTS_DIR/evaluation_summary.txt
        echo "运行时间: $(date)" >> $RESULTS_DIR/evaluation_summary.txt
        echo "==========================================" >> $RESULTS_DIR/evaluation_summary.txt
        echo "状态: 等待宿主机evo评估" >> $RESULTS_DIR/evaluation_summary.txt
        echo "宿主机评估脚本: $EVO_SCRIPT" >> $RESULTS_DIR/evaluation_summary.txt
        
    else
        echo "⚠️  未找到ground truth文件: $GT_TUM"
        echo "跳过evo评估"
    fi
    
else
    echo "=========================================="
    echo "PL-SLAM 运行失败!"
    echo "请检查日志文件: $RESULTS_DIR/plslam.log"
    exit 1
fi

echo "=========================================="
echo "运行完成!"
