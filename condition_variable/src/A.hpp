#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace CVtestA{

  class A
  {
      private:
        int test_;
        std::condition_variable cv_;

      public:
          A(int a):test_(a)
          {

          };
            
          ~A()
          {

          };

          inline std::condition_variable& getCV(){
          return cv_;
        }
        
        inline int& getTest(){
          return test_;
        }

        inline void sumA(int& fromB){
          test_ = test_ + fromB;
          cv_.notify_one();
        };
  };

} // namespace CVtestA
