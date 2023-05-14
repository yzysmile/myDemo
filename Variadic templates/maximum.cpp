// 变参模板 主要针对 可接受 任意类型 任意个数 的参数
#include <cmath>
#include <iostream>

int maximum(int n)
{
  return n;
}

template<typename... Args>
int maximum(int n, Args... agrs)
{
  return std::max(n, maximum(agrs...));
}

int main()
{
  std::cout << maximum(11, 22, 33, 44, 55, 66, 77, 88, 99) << std::endl;
  return 0;
}