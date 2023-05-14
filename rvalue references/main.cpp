#include <memory>
#include <cstring>

// 典型的右值： 临时对象
// 右值引用： 减少不必要的copy，可才用“偷”的方式进行构造。
// 对带有指针的类(对象)， 才有 使用 右值引用 的必要。
// 左值 转 右值： std::move(左值)，该左值 转成 右值 后，确保之后不再使用
// 完美转发： 当一个函数接受 右边引用参数类型时，在该函数内部再次调用 一个接受 右边引用参数类型的函数时候，
// 为了做到 接受的右值参数类型的不改变 必须使用 std::forward<>(右值)，

class MyString
{
  private:
     char* data_;
     int len_;
     
     // 深拷贝
     void init_data_(const char* s){
        data_ = new char[len_+1];
        std::memcpy(data_, s, len_);
        data_[len_] = '\0';
     }

  public:
    MyString():data_(nullptr), len_(0)
    {

    }

    MyString(const char* p):len_(std::strlen(p))
    {
      init_data_(p);
    }

    MyString(const MyString& str):len_(str.len_)
    {
        init_data_(str.data_);
    }
    
    MyString(MyString&& str) noexcept:data_(str.data_), len_(str.len_)
    {
        str.len_ = 0;
        str.data_ = nullptr;
    }

    MyString& operator=(const MyString& str)
    {   
        // 自我赋值 检查
        if(this != &str){
            if(data_){
              delete data_;
            }

            len_ = str.len_;
            init_data_(str.data_);
        } else{
          
        }
        return *this;
    }

    MyString& operator=(MyString&& str) noexcept
    {
       if(this != &str){
           if(data_){
              delete data_;
            }

            len_ = str.len_;
            data_ = str.data_;

            str.len_ = 0;
            str.data_ = nullptr;
       } else{

       }
       return *this;
    }
};