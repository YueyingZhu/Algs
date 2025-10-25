#!/bin/bash

# PL-SLAM CERBERUS 全序列运行脚本
# 使用方法: ./run_cerberus_all.sh [mode] [gui]

MODE=${1:-stereo}
GUI=${2:-false}

echo "=========================================="
echo "PL-SLAM CERBERUS 全序列运行脚本"
echo "模式: $MODE"
echo "GUI: $GUI"
echo "=========================================="

# 设置数据集路径和结果目录
DATASET_BASE="/datasets/CERBERUS"
RESULTS_BASE="/ws/results/CERBERUS/$MODE"

# 创建结果目录
mkdir -p $RESULTS_BASE

# 设置配置文件和词汇表路径
CONFIG_FILE="/ws/pl-slam/config/config/config_cerberus.yaml"

echo "数据集路径: $DATASET_BASE"
echo "结果目录: $RESULTS_BASE"
echo "配置文件: $CONFIG_FILE"

# 检查文件是否存在
if [ ! -d "$DATASET_BASE" ]; then
    echo "错误: 数据集路径不存在: $DATASET_BASE"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "错误: 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

# 获取所有序列
SEQUENCES=($(ls -d $DATASET_BASE/seq* 2>/dev/null | xargs -n 1 basename | sort -V))

if [ ${#SEQUENCES[@]} -eq 0 ]; then
    echo "错误: 未找到任何序列"
    exit 1
fi

echo "找到序列: ${SEQUENCES[*]}"
echo "=========================================="

# 存储评估结果
SUMMARY_FILE="$RESULTS_BASE/evaluation_summary.txt"
echo "PL-SLAM CERBERUS 评估结果" > $SUMMARY_FILE
echo "模式: $MODE" >> $SUMMARY_FILE
echo "运行时间: $(date)" >> $SUMMARY_FILE
echo "==========================================" >> $SUMMARY_FILE
printf "%-10s %-15s %-15s %-15s %-15s\n" "序列" "RMSE(m)" "Mean(m)" "Max(m)" "Min(m)" >> $SUMMARY_FILE
echo "----------------------------------------" >> $SUMMARY_FILE

# 处理每个序列
for seq in "${SEQUENCES[@]}"; do
    echo "处理序列: $seq"
    echo "=========================================="
    
    # 创建序列结果目录
    SEQ_RESULTS_DIR="$RESULTS_BASE/$seq"
    mkdir -p $SEQ_RESULTS_DIR
    
    # 运行单个序列
    if [ "$GUI" = "true" ]; then
        echo "启用GUI模式..."
        ./run_cerberus_single.sh $seq $MODE true
    else
        echo "无GUI模式..."
        ./run_cerberus_single.sh $seq $MODE false
    fi
    
    # 检查运行结果
    if [ $? -eq 0 ]; then
        echo "序列 $seq 运行完成!"
        
        # 评估序列
        echo "评估序列 $seq..."
        
        # 检查ground truth文件（优先使用svo2的gt_tum.txt，否则使用原始CSV转换）
        GT_TUM="/home/zyy/Algs/svo2/Results_svo2/CERBERUS/stereo+imu/${seq}/gt_tum.txt"
        GT_CSV="/datasets/CERBERUS/${seq}_gt.csv"
        
        # 如果svo2的gt_tum.txt不存在，则使用原始CSV转换
        if [ ! -f "$GT_TUM" ]; then
            echo "svo2 gt_tum.txt不存在，尝试使用原始CSV转换..."
            GT_TUM="$SEQ_RESULTS_DIR/gt_tum.txt"
            
            if [ -f "$GT_CSV" ]; then
                echo "转换ground truth格式..."
                python3 /ws/convert_gt.py "$GT_CSV" "$GT_TUM"
            fi
        fi
        
        if [ -f "$GT_TUM" ]; then
            echo "找到ground truth文件: $GT_TUM"
            
            # 使用evo评估
            export PATH=/home/dev4/.local/bin:$PATH
            cd $SEQ_RESULTS_DIR
            
            # 创建临时文件存储evo输出
            EVO_OUTPUT="$SEQ_RESULTS_DIR/evo_output.txt"
            EVO_ERROR="$SEQ_RESULTS_DIR/evo_error.txt"
            
            # 运行evo评估
            evo_ape tum "$GT_TUM" trajectory.txt --align --t_max_diff 0.2 2>$EVO_ERROR >$EVO_OUTPUT
            
            if [ $? -eq 0 ]; then
                # 提取关键指标（evo输出是tab分隔的）
                RMSE=$(grep "rmse" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
                MEAN=$(grep "mean" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
                MAX=$(grep "max" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
                MIN=$(grep "min" $EVO_OUTPUT | tr -s '\t' | cut -d$'\t' -f2)
                
                # 判断结果是否合理
                RMSE_NUM=$(echo $RMSE | sed 's/[^0-9.]//g')
                REASONABLE_THRESHOLD=1.0
                
                if (( $(echo "$RMSE_NUM < $REASONABLE_THRESHOLD" | bc -l) )); then
                    # 结果合理，保存evo结果
                    echo "✅ 序列 $seq 结果合理 (RMSE: $RMSE)"
                    
                    # 保存APE结果
                    cp $EVO_OUTPUT $SEQ_RESULTS_DIR/ape_results.txt
                    
                    # 尝试保存绘图（忽略错误）
                    evo_ape tum "$GT_TUM" trajectory.txt --align --t_max_diff 0.2 \
                        --plot --plot_mode xyz \
                        --save_results $SEQ_RESULTS_DIR/ape_results.zip \
                        --save_plot $SEQ_RESULTS_DIR/ape_plot.pdf 2>/dev/null || true
                    
                    # 尝试轨迹对比图
                    evo_traj tum trajectory.txt "$GT_TUM" \
                        --ref "$GT_TUM" \
                        --align --t_max_diff 0.2 \
                        --plot --plot_mode xyz \
                        --save_plot $SEQ_RESULTS_DIR/traj_plot.pdf 2>/dev/null || true
                    
                    # 创建评估总结
                    echo "PL-SLAM CERBERUS 评估结果" > $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "序列: $seq" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "模式: $MODE" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "运行时间: $(date)" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "==========================================" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "RMSE: $RMSE m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "Mean: $MEAN m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "Max:  $MAX m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "Min:  $MIN m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "状态: 合理 ✅" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    
                    # 记录到总结文件
                    printf "%-10s %-15s %-15s %-15s %-15s\n" "$seq" "$RMSE" "$MEAN" "$MAX" "$MIN" >> $SUMMARY_FILE
                    
                else
                    # 结果不合理
                    echo "❌ 序列 $seq 结果不合理 (RMSE: $RMSE >= $REASONABLE_THRESHOLD)"
                    
                    # 保存错误信息
                    echo "PL-SLAM CERBERUS 评估结果" > $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "序列: $seq" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "模式: $MODE" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "运行时间: $(date)" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "==========================================" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "RMSE: $RMSE m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "Mean: $MEAN m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "Max:  $MAX m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "Min:  $MIN m" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "状态: 不合理 ❌" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    echo "建议: 检查标定文件和配置参数" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                    
                    # 记录到总结文件
                    printf "%-10s %-15s %-15s %-15s %-15s\n" "$seq" "$RMSE" "$MEAN" "$MAX" "$MIN" >> $SUMMARY_FILE
                fi
                
            else
                echo "❌ 序列 $seq evo评估失败"
                cat $EVO_ERROR
                
                # 保存错误信息
                echo "PL-SLAM CERBERUS 评估结果" > $SEQ_RESULTS_DIR/evaluation_summary.txt
                echo "序列: $seq" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                echo "模式: $MODE" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                echo "运行时间: $(date)" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                echo "==========================================" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                echo "状态: evo评估失败 ❌" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                echo "错误信息:" >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                cat $EVO_ERROR >> $SEQ_RESULTS_DIR/evaluation_summary.txt
                
                # 记录到总结文件
                printf "%-10s %-15s %-15s %-15s %-15s\n" "$seq" "ERROR" "ERROR" "ERROR" "ERROR" >> $SUMMARY_FILE
            fi
        else
            printf "%-10s %-15s %-15s %-15s %-15s\n" "$seq" "NO_GT" "NO_GT" "NO_GT" "NO_GT" >> $SUMMARY_FILE
            echo "序列 $seq 无ground truth文件: $GT_TUM"
        fi
        
    else
        echo "序列 $seq 运行失败!"
        printf "%-10s %-15s %-15s %-15s %-15s\n" "$seq" "FAILED" "FAILED" "FAILED" "FAILED" >> $SUMMARY_FILE
    fi
    
    echo "=========================================="
done

echo "=========================================="
echo "全序列运行完成!"
echo "评估结果保存在: $SUMMARY_FILE"
echo "=========================================="

# 显示总结
echo "评估总结:"
cat $SUMMARY_FILE
