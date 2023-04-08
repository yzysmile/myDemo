# include "B.hpp"
# include <chrono>

int main()
{
   CVtestB::B b;
   std::chrono::milliseconds timeout = std::chrono::milliseconds(50);
   
   int result = b.sumB(timeout);
   std::cout<< "main success";
   return result;
}