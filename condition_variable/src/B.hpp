#include <iostream>
#include <thread>
#include <mutex>

#include "A.hpp"

/**
 * @brief 在B类中委托A的对象，B对象的成员函数 在独立线程 调用 成员变量A对象 的成员函数
 *        A对象 的成员函数 计算完毕之前，阻塞50milliseconds； 计算完毕后，发出通知，不再阻塞。
 */

namespace CVtestB{
    class B
    {
    private:
    std::shared_ptr<CVtestA::A> obj_a_ {nullptr};
    int sum_{};

    public:
        B()
        {   
            int a = 1;
            obj_a_ = std::make_shared<CVtestA::A>(a);
        };

        ~B()
        {

        };

        inline int& sumB(std::chrono::milliseconds& timeout){
            int fromB = 2;
            std::thread myThread(&CVtestA::A::sumA, obj_a_, std::ref(fromB));
            myThread.detach();

            std::mutex mutex_wait;
            std::unique_lock<std::mutex> wait(mutex_wait);
            std::cv_status cvsts = obj_a_->getCV().wait_for(wait, timeout);
            if(cvsts == std::cv_status::timeout){
                std::cout << "timeout";
                return sum_;
            } else{
                std::cout << "success";
                return obj_a_->getTest();
            }
            wait.unlock();
        };

    };
} // namespace CVtestB
