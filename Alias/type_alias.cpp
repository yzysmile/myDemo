// using 与 函数指针 联合使用
using func = void(*) (int, int);

void example(int, int)
{
  return;
}

func fn = example;