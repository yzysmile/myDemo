# include "producer.cpp"
# include <thread>

int main()
{
  promiseFutureProducer::producer<int> p(9, 5);
  
  // std::thread 调用类成员函数
  // 假设要多线程调用T类型obj对象的func函数：
  // std::thread(&T::func, &obj, 参数);
  std::thread calculate_thread(&promiseFutureProducer::producer<int>::complicatedCompute, &p);
  std::cout << "result: 9*8*7*6*5:" << p.get_consumer().get_future().get();
  calculate_thread.join();
  return 0;
}
