# include "producer.cpp"
# include <thread>

int main()
{
  promiseFutureProducer::producer<int> p(9, 5);
  // promise_future
    // std::thread 调用类成员函数
    // 假设要多线程调用T类型obj对象的func函数，其格式为：
    // std::thread(&T::func, &obj, 参数);
  std::thread calculate_thread(&promiseFutureProducer::producer<int>::complicatedCompute, &p);
  calculate_thread.detach();
  std::cout << "result: 9*8*7*6*5 is: "<< p.get_consumer().get_future().get() << std::endl;
  std::cout << "check future is valid or not after future get result " << p.get_consumer().get_future().valid() << std::endl;
  
  
//   std::packaged_task<int()>task([](){return 1;});
//   std::thread t1(std::ref(task));
//   t1.detach();
//   std::future<int> f = task.get_future();
//   auto result1 = f.get();
//   std::cout << result1 << std::endl;

// std::packaged_task
  // 1.packaged_task类对象中 中包含了生产线程的生产函数，2.无需 在生产线程的生产函数中执行 packaged_task类对象.set_value().
    // std::thread calculate_thread1(std::ref(promiseFutureConsumer::consumer<int>::global_task));
    // std::cout << "result1: 9*8*7*6*5 is: "<< p.get_consumer().get_taskFuture().get() << std::endl;
    // std::cout << "check taskFuture_ is valid or not after taskFuture_ get result " << p.get_consumer().get_taskFuture().valid() << std::endl;
    // calculate_thread1.detach();

  // std::async
   // 1.自动将thread、生产线程的生产函数 绑定在一起 ， 2.自动将future与promise绑定在一起 且 3.无需在生产线程的生产函数中执行 promise类对象.set_value().
   // auto func = std::bind(&promiseFutureProducer::producer<int>::computeWithPackagedTaskAndAsync, &p);
   // p.get_consumer().get_asyncFuture() = std::async(std::launch::async, func);
  std::cout << "asyncFuture value before launch async: " <<p.get_consumer().get_asyncFuture().valid()<< std::endl;
   
  p.get_consumer().get_asyncFuture() = std::async(std::launch::async, &promiseFutureProducer::producer<int>::computeWithPackagedTaskAndAsync, &p);

  std::cout << "asyncFuture value after launch async: " <<p.get_consumer().get_asyncFuture().valid()<< std::endl;

  std::cout << "result2: 9*8*7*6*5 is: " << p.get_consumer().get_asyncFuture().get() << std::endl;

  std::cout << "asyncFuture value after get value: " <<p.get_consumer().get_asyncFuture().valid()<< std::endl;

  return 0;
}
