#include <iostream>
#include <type_traits>
#include <utility>

// int countMatchElements(int *begin, int *end){
//     int result = 0;
//     for(; begin != end; ++begin){
//         if(*begin > 20){
//             ++result;
//         }
//     }
//     return result;
// }

// 第三个函数参数 为 函数指针，传入的函数 接受一个参数类型为 const int &， 返回值为 bool的函数
// 这里同时可以传入 std::function<bool(const int &)>
// int countMatchElements(int *begin, int *end, bool(*pred)(const int &)){
//   int result = 0;
//   for(; begin != end; ++begin){
//      if( pred(*begin) ){
//         ++result;
//      }
//   }
// }

bool isGreater20(const int &val) { return val > 20; }
bool isGreater25(const int &val) { return val > 25; }
bool isLess10(const int &val) {  return val < 10;}

// 在传入函数指针的情况下， 将其改造为 模板函数，可以同时处理 int 和 string
template<typename T>
int countMatchElements(T *begin, T *end, bool(*pred)(const T &)){
    int result = 0;
    for(; begin != end; ++begin){
        if( pred(*begin) ){
            ++result;
        }  
    }
}

bool isTinyStr(const std::string &val) { return val.size() <= 3;}

// isGreater20, isGreater25, isLess10 将其 参数(20,25,10) 写死, 为了 解决这个弊端， 引入 “仿函数”
// 仿函数往往是一个 struct，其中重载“()”( operator() )以实现仿函数的调用
template<typename T>
struct Greater{
    T val_;
    explicit Greater(T value): val_(value) {}
    bool operator()(const T &val) const { return val>val_; }
};

// 但函数指针 无法 接受 仿函数 的传入， 解决该问题的方式之一是： 将模板进行到底
// typename Pred 暗示传入的模板类型 是一个 用于评判标准的函数
template<typename T, typename Pred>
int countMatchElements(T *begin, T *end, Pred functor){
    int result = 0;
    for(; begin != end; ++begin){
     if(functor(*begin)){
        ++result;
     }
    }
    return result;
}


int main(){
    int intArray[] = { 11, 16, 21, 19, 17, 30};

    // // 统计大于20的元素数量
    //  // int result = countMatchElements(intArray, intArray + sizeof(intArray)/sizeof(int));
    //     // 函数名称就是函数地址
    //     int greater20Count = countMatchElements(intArray, intArray + sizeof(intArray)/sizeof(int), isGreater20);

    // // 统计小于10的元素数量
    //     int less10Count = countMatchElements(intArray, intArray + sizeof(intArray)/sizeof(int), isLess10);
    
    // // 统计小于10的元素数量
    //     int less10Count = countMatchElements(intArray, intArray + sizeof(intArray)/sizeof(int), isLess10);

    // std::cout << greater20Count << std::endl;
    
    // std::string stringArray[] = { "the", "template", "is", "a", "useful", "tool"};
    
    // // 统计size()小于3的字符数量
    //     int sizeLess3Count = countMatchElements(stringArray, stringArray + sizeof(stringArray)/sizeof(std::string), isTinyStr);
    
    // 传入仿函数(以防止参数写死) 以 统计大于20的元素数量
        Greater<int> greater20functor(20);
        int greater20CountByFunctor = countMatchElements(intArray, intArray + sizeof(intArray)/sizeof(int), greater20functor);
    
    // 以Lambada表达式表现, 在lambda中使用auto在c++14后支持
        auto greater20 = [](auto &val) -> bool { return val > 20; }; 
        int greater20CountByLambda = countMatchElements(intArray, intArray + sizeof(intArray)/sizeof(int), greater20functor);
    
    return 0;
}