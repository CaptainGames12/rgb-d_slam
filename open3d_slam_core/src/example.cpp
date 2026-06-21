#include <iostream>
#include <memory>
#include <vector>
#include <string>

// ЗАМІНА: Замість #include <ros/package.h> використовуємо ament_index_cpp
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <open3d_slam/configuration_file_resolver.h>
#include <open3d_slam/lua_parameter_dictionary.h>

int main(int argc, char** argv) {
  using namespace lua_dict;

  // ЗАМІНА: Отримуємо шлях до share-директорії пакета у ROS 2 workspace
  // Переконайтеся, що папка config копіюється у share/ вашого пакета через CMakeLists.txt
  const std::string path = ament_index_cpp::get_package_share_directory("lua_parameter_dictionary") + "/config/";
  
  const std::vector<std::string> paths({path});
  auto fileResolver = std::make_unique<ConfigurationFileResolver>(paths);

  const std::string fullPath = fileResolver->GetFullPathOrDie("example.lua");
  const std::string fullContent = fileResolver->GetFileContentOrDie("example.lua");

  std::cout << "Loaded full path: " << fullPath << std::endl;
  std::cout << "Loaded content: " << fullContent << "\n \n";

  LuaParameterDictionary dict(fullContent, std::move(fileResolver));

  // Подальша логіка роботи зі словником залишається незмінною...
  std::cout << "Has key example_struct: " << std::boolalpha << dict.HasKey("example_struct") << "\n";
  std::cout << "some_double_shizzle=" << dict.GetDouble("some_double_shizzle") << "\n";

  return 0;
}