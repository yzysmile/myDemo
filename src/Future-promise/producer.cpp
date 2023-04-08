# include <iostream>
# include <future>
# include <thread>
# include <memory>
# include <functional>

# include "consumer.cpp"

namespace promiseFutureProducer{

template <typename Ty>
class producer
{
public:
    producer(Ty bigger,Ty smaller):bigger_(bigger),smaller_(smaller),
    func_(std::bind(&producer::computeWithPackagedTaskAndAsync, this))
    {
      c_.task_bind_func(func_);
    }

    ~producer()
    {

    }
    
    // 生产线程生产future最终获得结果的函数
    void complicatedCompute()
    {                          
      Ty result = bigger_; 
      while(bigger_ > smaller_){
        result =  result * (bigger_ - 1);
        bigger_--;
      }
      c_.get_promise().set_value(result);
    }
    
    promiseFutureConsumer::consumer<Ty>& get_consumer()
    {
      return c_;
    }
    
    // 生产线程生产future最终获得结果的函数
    Ty computeWithPackagedTaskAndAsync()
    {
      Ty result = bigger_; 
      while(bigger_ > smaller_){
        result =  result * (bigger_ - 1);
        bigger_--;
      }
      std::cout << result << std::endl;
      return result;
    }
    
private:
   Ty bigger_;
   Ty smaller_;
   promiseFutureConsumer::consumer<Ty> c_;
   
   // std::function是 函数指针 的“超集”
   // 函数指针 只能指向 全局函数 或 静态函数
   // std::function 可以指向全局函数、静态函数、类的成员函数(与std::bind配合使用)
    // std::bind的作用：
     // 1.将可调用对象和其参数绑定成一个防函数;
     // 2.只绑定部分参数，减少可调用对象传入的参数。
   std::function<Ty ()> func_;
};

}  //  promiseFutureProducer
