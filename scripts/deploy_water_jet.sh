src=$1
ws=$2
echo "Source directory: ${src}"
echo "Workspace directory: ${ws}"
deploy_dir=${ws}/src/$3

# 将src目录下的所有.py文件的可执行权限设置为+x
find ${src} -name "*.py" -exec chmod +x {} \;

# 将src目录下的所有.sh文件的可执行权限设置为+x
find ${src} -name "*.sh" -exec chmod +x {} \;

# 将src目录下的除忽略列表的所有内容链接到目标工作空间目录

# 获取ws上一级目录中的目录名作为忽略列表
ignore_list=()
for dir in ${ws}/src/*; do
    echo ${dir}
    if [ -d "${dir}" ]; then
        ignore_list+=("$(basename "${dir}")")
    fi
done


# 删除已存在的符号链接
rm -r ${deploy_dir}

# 确保目标工作空间目录存在
mkdir -p ${deploy_dir}

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
            ln -s ${folder} ${deploy_dir}/${folder_name}
        else
            echo "Skipping: ${folder_name}"
        fi
    fi
done
