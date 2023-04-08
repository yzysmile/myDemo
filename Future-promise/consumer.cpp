# include <iostream>
# include <future>
# include <thread>

// 消费线程：创建 promise@future 对象，并将两者绑定
//         并将promise对象 以引用的方式 传递给 生产线程；

// 生产线程 负责产生结果 promise.set_value(...)， 
// 消费线程在生产线程产生结果前 future.get()阻塞
namespace promiseFutureConsumer{

template <typename Ty>
class consumer
{
 public:
  consumer()
  {
    // 绑定
    future_ = promise_.get_future();
  }

  ~consumer()
  {

  }
  
  std::promise<Ty>& get_promise()
  {
    return promise_;
  }

  std::future<Ty>& get_future()
  {
    return future_;
  }

 private:
    std::promise<Ty> promise_;
    std::future<Ty> future_;
};

}  //  futurePromise