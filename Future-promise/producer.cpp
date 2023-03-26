# include <iostream>
# include <future>
# include <thread>
# include <memory>

# include "consumer.cpp"

namespace promiseFutureProducer{

template <typename Ty>
class producer
{
public:
    producer(Ty bigger,Ty smaller):bigger_(bigger),smaller_(smaller)
    {
      
    }
    ~producer()
    {
        
    }
    
    void complicatedCompute()
    {                          
      Ty result = bigger_; 
      while(smaller_ >= bigger_){
        result =  result * (bigger_ - 1);
        bigger_--;
      }
      c_.get_promise().set_value(result);
    }
    
    promiseFutureConsumer::consumer<int>& get_consumer()
    {
      return c_;
    }

private:
   Ty bigger_;
   Ty smaller_;
   promiseFutureConsumer::consumer<int> c_;
};

}  //  promiseFutureProducer
