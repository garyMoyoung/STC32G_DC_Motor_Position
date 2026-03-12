#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将文件编码转换为GBK编码，支持多种输入编码检测
"""

import os
import glob

# 需要转换的文件扩展名
extensions = ['*.c', '*.h']

# 获取脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# 需要尝试的编码列表
encodings_to_try = ['gbk', 'gb2312', 'utf-8', 'cp1252', 'latin-1']

# 查找所有需要转换的文件
files_to_convert = []
for ext in extensions:
    files_to_convert.extend(glob.glob(ext))

print(f"找到 {len(files_to_convert)} 个文件需要转换编码")

for file_path in files_to_convert:
    content = None
    detected_encoding = None
    
    # 尝试不同的编码来读取文件
    for encoding in encodings_to_try:
        try:
            with open(file_path, 'rb') as f:
                raw_bytes = f.read()
            content = raw_bytes.decode(encoding)
            detected_encoding = encoding
            break
        except (UnicodeDecodeError, LookupError):
            continue
    
    if content is None:
        print(f"✗ 无法确定 {file_path} 的编码")
        continue
    
    try:
        # 以GBK编码写回文件
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✓ 已转换: {file_path} (原编码: {detected_encoding} -> GBK)")
    except Exception as e:
        print(f"✗ 转换 {file_path} 时出错: {e}")

print("\n编码转换完成！")
