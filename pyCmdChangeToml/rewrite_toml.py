import toml
import argparse
import os
# # 读取toml
# file_a = "launch/config/demo.toml"

# if __name__ == "__main__":
#     a = toml.load(file_a)
#     print("变量a的类型: %s" % type(a))
#     print("变量a的内容: %s" % a)

# 传入参数
""" 改变toml配置文件 """
 # argparse对象 具有 将命令行解析成 Python 数据类型 的功能
 # argparse.ArgumentParser(prog=None, usage=None, description=None, epilog=None)
parser = argparse.ArgumentParser(description="This is a program for replacing params in the toml.")

# 用于向解析器中添加一个选项
# required 设为 True 则代表此选项为必选项
parser.add_argument('--a', type=str, default="1")
parser.add_argument('--b', type=str, default="70")
args = parser.parse_args()

a, b = args.a, args.b

wife_name = "cyberdog" + a
register_address = "192.168.31." + b + ":50053"

# 写toml
# dst_file = "/home/yzyrobot/myself_demo/src/pyCmdChangeToml/demo.toml"
dst_file = "/opt/ros2/cyberdog/share/cyberdog_aft/config/aft_register_uri.toml"

if __name__ == "__main__":
    a = {'address': {'wifi_name': wife_name,
                     'register_address': register_address},
        }
    
    # a = {'a': 1, 
    #      'b': {'c': 1, 'd': {'e': 1}}
    #      }
    
    with open(dst_file, 'w') as f:
        r = toml.dump(a, f)
        print(r)
