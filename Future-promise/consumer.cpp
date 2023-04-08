# include <iostream>
# include <future>
# include <thread>

// 消费线程：创建 promise & future 对象，并将两者绑定;
//         promise保存1个共享状态的值,future.get()此值。
//         
//         并将promise对象 以引用的方式 传递给 生产线程；
//         消费线程 在 生产线程 产生结果前 future.get()阻塞

// 生产线程 负责产生结果 promise.set_value(...)， 
namespace promiseFutureConsumer{

template <typename Ty>
class consumer
{
 public:
  consumer()
  {
    std::cout << "check future is valid or not before bond with promise: " << future_.valid() << std::endl;
    // 通过promise的成员函数get_future，将future、promise绑定在一起。
    future_ = promise_.get_future();
    std::cout << "check future is valid or not after bond with promise: " << future_.valid()<< std::endl;
    
    // 绑定
    // taskFuture_ = task_.get_future();
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

  void task_bind_func(std::function<Ty ()> func)
  {
    static std::packaged_task<Ty()> global_task(func);
    // 这样写为什么不行？
    // task_(func);
    std::cout << "check taskFuture_ is valid or not before bond with task_ " << taskFuture_.valid() << std::endl;
    // 通过packaged_task的成员函数，将future、packaged_task绑定在一起
    taskFuture_ = global_task.get_future();
    // taskFuture_ = task_.get_future();
    std::cout << "check taskFuture_ is valid or not after bond with task_ " << taskFuture_.valid() << std::endl;
  }
 
  std::future<Ty>& get_taskFuture()
  {
    return taskFuture_;
  }

  std::future<Ty>& get_asyncFuture()
  {
    return asyncFuture_;
  }
 private:
    std::promise<Ty> promise_;
    std::future<Ty> future_; 

    std::packaged_task<Ty()> task_;
    std::future<Ty> taskFuture_; 

    std::future<Ty> asyncFuture_; 
};

}  //  futurePromise