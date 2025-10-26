#ifndef MY_PACKAGE__MY_HEADER_HPP_
#define MY_PACKAGE__MY_HEADER_HPP_

namespace my_package
{

/**
 * @brief Example header file for the package
 */
class MyClass
{
public:
  MyClass() = default;
  ~MyClass() = default;

  /**
   * @brief Example method
   * @return A greeting string
   */
  const char* greet() const
  {
    return "Hello from my_package!";
  }
};

}  // namespace my_package

#endif  // MY_PACKAGE__MY_HEADER_HPP_
