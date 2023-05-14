#include <vector>
#include <queue>
#include <iostream>

// template function
template<typename Container>
void test_moveable(Container c)
{
  typedef typename std::iterator_traits<typename Container::iterator>::value_type Valtype;
  
  for(int i = 0; i < 10; ++i){
    c.insert(c.end(), Valtype());
  }
  
  Container c1(c);
  Container c2(std::move(c));
  std::cout<< "finish" << std::endl;
}


// template alias:给 模板 起别名

// template template parameter 联合template alias使用
template<typename T>
using Vec = std::vector<T, std::allocator<T>>;

template<typename T, template<class>
                      class Container >
class XCLs
{
private:
  Container<T> c;

public:
  XCLs(){
    for (int i = 0; i < 10; i++){
      c.insert(c.end(), T());
    }
    Container<T> c1(c);
    Container<T> c2(std::move(c));
    std::cout<< "construct finish" << std::endl;
  }
};



int main()
{
  test_moveable(std::vector<int>());

  XCLs<int, Vec> C;
  return 0;
}