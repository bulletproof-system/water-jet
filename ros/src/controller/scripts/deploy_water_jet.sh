echo $0
src=$(dirname $(dirname $(dirname $0)))
echo ${src}

# 将src目录下的除忽略列表的所有内容链接到目标工作空间目录
ws=/home/robot/catkin_ws/src/fri_g3_morning
# ws=/home/ltt/download/src/fir_g3_morning

# 定义一个数组，包含要忽略的目录名称
ignore_list=("wpb_home" "wpr_simulation" "nav_pkg")

# 删除已存在的符号链接
rm -r ${ws}

# 确保目标工作空间目录存在
mkdir -p ${ws}

for folder in ${src}/*; do
    if [ -d "${folder}" ]; then  # 检查是否为目录
        # 获取文件夹名称，即去掉前面的路径
        folder_name=${folder##*/}
        # 检查文件夹名称是否在忽略列表中
        is_ignored=false
        for ignore in "${ignore_list[@]}"; do
            if [ "${folder_name}" == "${ignore}" ]; then
                is_ignored=true
                break
            fi
        done

        if [ "${is_ignored}" = false ]; then
            echo "Linking: ${folder}"
            # 创建符号链接
            ln -s ${folder} ${ws}/${folder_name}
        else
            echo "Skipping: ${folder_name}"
        fi
    fi
done

# ln -s ${src}/controller ${ws}/controller
# ln -s ${src}/create_map ${ws}/create_map
# ln -s ${src}/navigation ${ws}/navigation
# ln -s ${src}/world_simulation ${ws}/world_simulation
# ln -s ${src}/pot_database ${ws}/pot_database
# ln -s ${src}/object_detect ${ws}/object_detect
# ln -s ${src}/robot_arm ${ws}/robot_arm
# ln -s ${src}/tf2_web_republisher ${ws}/tf2_web_republisher